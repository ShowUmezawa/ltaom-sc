// Exercises the actual IMU prediction + deskew path against analytic motion.
#include "IMU_Processing.hpp"
#include <iostream>
#include <stdexcept>

using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void no_lidar_update(state_ikfom&, esekfom::dyn_share_datastruct<double>&) {}

struct ImuProcessBiasRegression {
  static sensor_msgs::ImuConstPtr sample(double time, const V3D& gyro, const V3D& acc) {
#ifdef LTAOM_TEST_ROS2
    auto m = std::make_shared<sensor_msgs::Imu>();
    m->header.stamp = ltaom_ros2::Time().fromSec(time);
#else
    sensor_msgs::Imu::Ptr m(new sensor_msgs::Imu());
    m->header.stamp.fromSec(time);
#endif
    m->angular_velocity.x=gyro.x();m->angular_velocity.y=gyro.y();m->angular_velocity.z=gyro.z();
    m->linear_acceleration.x=acc.x();m->linear_acceleration.y=acc.y();m->linear_acceleration.z=acc.z();
    return m;
  }

  static void run(const char* name, V3D bg, V3D ba, double scale, double yaw_rate, V3D velocity) {
    Filter filter;
    double limits[23];std::fill(limits,limits+23,1e-6);
    filter.init_dyn_share(get_f,df_dx,df_dw,no_lidar_update,3,limits);
    state_ikfom initial;
    initial.pos=V3D::Zero();initial.vel=velocity;initial.bg=bg;initial.ba=ba;
    initial.rot=SO3(M3D::Identity());
    initial.grav=S2(V3D(0,0,-G_m_s2));
    initial.offset_R_L_I=SO3(Eigen::AngleAxisd(.4,V3D::UnitZ()).toRotationMatrix());
    initial.offset_T_L_I=V3D(.171,0,.0908);
    filter.change_x(initial);
    const V3D gravity(initial.grav[0],initial.grav[1],initial.grav[2]);
    // Biases are expressed in the gravity-normalized accelerometer units, as in get_f().
    const V3D raw_acc=(-gravity+ba)/scale;
    const V3D raw_gyro=V3D(0,0,yaw_rate)+bg;
    ImuProcess imu;
    imu.mean_acc=V3D(0,0,G_m_s2/scale);
    imu.last_lidar_end_time_=100.;imu.first_lidar_time=100.;
    imu.last_imu_=sample(100.,raw_gyro,raw_acc);
    imu.acc_s_last=V3D::Zero();imu.angvel_last=V3D(0,0,yaw_rate);
    imu.set_acc_cov(V3D::Constant(.1));imu.set_gyr_cov(V3D::Constant(.1));
    imu.cov_acc=imu.cov_acc_scale;imu.cov_gyr=imu.cov_gyr_scale;
    MeasureGroup measurements;
    measurements.lidar_beg_time=100.;measurements.lidar_end_time=100.1;
    measurements.lidar.reset(new PointCloudXYZI());
    const V3D world_point(5.,2.,1.);
    for(int k=1;k<=20;++k) {
      const double t=k*.005;
      measurements.imu.push_back(sample(100.+t,raw_gyro,raw_acc));
      M3D rotation=Eigen::AngleAxisd(yaw_rate*t,V3D::UnitZ()).toRotationMatrix();
      V3D local=initial.offset_R_L_I.conjugate()*(rotation.transpose()*(world_point-velocity*t)-initial.offset_T_L_I);
      PointType p{};p.x=local.x();p.y=local.y();p.z=local.z();p.curvature=t*1000.;
      measurements.lidar->push_back(p);
    }
    PointCloudXYZI deskewed;
    imu.UndistortPcl(measurements,filter,deskewed);
    const auto result=filter.get_x();
    const M3D rotation=Eigen::AngleAxisd(yaw_rate*.1,V3D::UnitZ()).toRotationMatrix();
    const V3D expected=initial.offset_R_L_I.conjugate()*(rotation.transpose()*(world_point-velocity*.1)-initial.offset_T_L_I);
    double cloud_error=0;
    for(const auto& p:deskewed)cloud_error=std::max(cloud_error,(V3D(p.x,p.y,p.z)-expected).norm());
    if(cloud_error>2e-6) {
      std::cerr<<"expected "<<expected.transpose()<<"\n";
      for(const auto& point:deskewed)std::cerr<<point.curvature<<" "<<point.x<<" "<<point.y<<" "<<point.z<<"\n";
    }
    const double rot_error=(result.rot.toRotationMatrix()-rotation).norm();
    const double vel_error=(result.vel-velocity).norm();
    const double pos_error=(result.pos-velocity*.1).norm();
    std::cout<<name<<" rotation="<<rot_error<<" velocity="<<vel_error<<" position="<<pos_error<<" deskew="<<cloud_error<<std::endl;
    require(result.pos.allFinite()&&filter.get_P().allFinite(),"nonfinite state or covariance");
    require(rot_error<1e-8,"biased orientation propagation");
    require(vel_error<1e-8,"biased velocity propagation / incorrect acceleration scaling order");
    require(pos_error<1e-8,"biased position propagation");
    require(cloud_error<2e-6,"deskew does not agree with analytic rigid motion");
  }
};

int main(int argc,char** argv) {
#ifdef LTAOM_TEST_ROS2
  ltaom_ros2::init(argc,argv,"imu_bias_regression");
#else
  ros::init(argc,argv,"imu_bias_regression",ros::init_options::AnonymousName);ros::Time::init();
#endif
  int failures=0;
  const auto test=[&](const char* name,V3D bg,V3D ba,double scale,double yaw,V3D vel) {
    try {ImuProcessBiasRegression::run(name,bg,ba,scale,yaw,vel);}
    catch(const std::exception& e){std::cerr<<"FAIL "<<name<<": "<<e.what()<<std::endl;++failures;}
  };
  test("zero_bias_stationary",V3D::Zero(),V3D::Zero(),1.,0.,V3D::Zero());
  test("gyro_bias_stationary",V3D(.01,-.02,.03),V3D::Zero(),1.,0.,V3D::Zero());
  test("accel_bias_stationary",V3D::Zero(),V3D(.2,-.1,.3),1.,0.,V3D::Zero());
  test("scaled_accel_bias",V3D::Zero(),V3D(.2,-.1,.3),1.2,0.,V3D::Zero());
  test("biased_rotation_translation",V3D(.01,-.02,.03),V3D(.2,-.1,.3),1.2,.5,V3D(1.,-.2,0));
#ifdef LTAOM_TEST_ROS2
  ltaom_ros2::shutdown();
#else
  ros::shutdown();
#endif
  std::cout<<(failures?"FAILED":"PASSED")<<" IMU bias prediction and deskew: "<<failures<<" failing cases"<<std::endl;
  return failures?1:0;
}
