#pragma once
// Native rclcpp transport facade for the existing single-threaded algorithm.
// No ROS1 runtime, roscpp ABI, bridge or ROS1 message serialization is used.
#include <rclcpp/rclcpp.hpp>
#include <ltaom_transport/queue_budget.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <ltaom_interfaces/msg/submap.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ltaom_interfaces/msg/pose6_d.hpp>
#include <ltaom_interfaces/msg/states.hpp>
#include <ltaom_interfaces/msg/custom_msg.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cassert>
#include <deque>
#include <algorithm>
#include <boost/make_shared.hpp>
#include <cmath>
#include <functional>
#include <typeindex>
#include <type_traits>
namespace sensor_msgs {using PointCloud2=msg::PointCloud2;using PointField=msg::PointField;using Imu=msg::Imu;
using PointCloud2Ptr=PointCloud2::SharedPtr;using PointCloud2ConstPtr=PointCloud2::ConstSharedPtr;using ImuConstPtr=Imu::ConstSharedPtr;}
namespace nav_msgs {using Odometry=msg::Odometry;using Path=msg::Path;using OdometryPtr=Odometry::SharedPtr;using OdometryConstPtr=Odometry::ConstSharedPtr;}
namespace geometry_msgs {using PoseWithCovariance=msg::PoseWithCovariance;using Pose=msg::Pose;using PoseStamped=msg::PoseStamped;using Vector3=msg::Vector3;using Quaternion=msg::Quaternion;using Point=msg::Point;}
namespace std_msgs {using String=msg::String;using Int32=msg::Int32;using Int32ConstPtr=Int32::ConstSharedPtr;using UInt64=msg::UInt64;
using Float32MultiArray=msg::Float32MultiArray;using Float64MultiArray=msg::Float64MultiArray;
using Float32MultiArrayPtr=Float32MultiArray::SharedPtr;using Float32MultiArrayConstPtr=Float32MultiArray::ConstSharedPtr;
using Float64MultiArrayPtr=Float64MultiArray::SharedPtr;using Float64MultiArrayConstPtr=Float64MultiArray::ConstSharedPtr;}
namespace visualization_msgs {using Marker=msg::Marker;using MarkerArray=msg::MarkerArray;}
namespace fast_lio {using Pose6D=ltaom_interfaces::msg::Pose6D;using States=ltaom_interfaces::msg::States;}
namespace livox_ros_driver {using CustomMsg=ltaom_interfaces::msg::CustomMsg;}
namespace std_srvs {using Trigger=srv::Trigger;}
namespace ltaom_ros2 {
inline rclcpp::Node::SharedPtr node;
inline std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
inline std::vector<std::function<void()>> release_entities;
inline std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
inline bool isInitialized(){return bool(node);}
inline void init(int argc,char** argv,const std::string& name){
 rclcpp::init(argc,argv);node=std::make_shared<rclcpp::Node>(name,rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
 executor=std::make_shared<rclcpp::executors::SingleThreadedExecutor>();executor->add_node(node);
}
inline bool ok(){return rclcpp::ok();}
inline void spinOnce(){try{executor->spin_some();}catch(const ltaom_budget::Overflow& e){RCLCPP_ERROR(node->get_logger(),"%s",e.what());}ltaom_budget::write();}
inline void shutdown(){
 executor->cancel();tf_broadcaster.reset();
 for(auto& release:release_entities)release();release_entities.clear();
 executor->remove_node(node);executor.reset();node.reset();rclcpp::shutdown();
}
using Rate=rclcpp::WallRate;
inline std::string parameterName(std::string s){while(!s.empty()&&s.front()=='/')s.erase(0,1);std::replace(s.begin(),s.end(),'/','.');return s;}
template<class T> bool getParam(const std::string& name,T& value){
 const auto key=parameterName(name);
 if(!node->has_parameter(key))return false;
 const auto p=node->get_parameter(key);
 if constexpr(std::is_floating_point_v<T>) {
  value=static_cast<T>(p.get_type()==rclcpp::ParameterType::PARAMETER_INTEGER?double(p.as_int()):p.as_double());
 } else if constexpr(std::is_same_v<T,int>) {
  value=static_cast<int>(p.as_int());
 } else if constexpr(std::is_same_v<T,std::vector<double>>) {
  if(p.get_type()==rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY){const auto v=p.as_integer_array();value.assign(v.begin(),v.end());}
  else value=p.as_double_array();
 } else value=p.get_value<T>();
 return true;
}
namespace param {template<class T>bool get(const std::string& n,T& v){return getParam(n,v);}}
struct Time {
 int64_t ns=0;
 Time()=default;Time(const builtin_interfaces::msg::Time& t):ns(rclcpp::Time(t).nanoseconds()){}
 static Time now(){Time t;t.ns=node->now().nanoseconds();return t;}
 Time& fromSec(double s){const double whole=std::floor(s);ns=static_cast<int64_t>(whole)*1000000000LL+static_cast<int64_t>(std::llround((s-whole)*1e9));return *this;}
 Time& fromNSec(uint64_t n){ns=n;return *this;}
 double toSec()const{return double(ns/1000000000LL)+double(ns%1000000000LL)*1e-9;}uint64_t toNSec()const{return ns;}
 operator builtin_interfaces::msg::Time()const{return rclcpp::Time(ns).operator builtin_interfaces::msg::Time();}
};
inline double seconds(const builtin_interfaces::msg::Time& t){return double(t.sec)+double(t.nanosec)*1e-9;}
inline uint64_t nanoseconds(const builtin_interfaces::msg::Time& t){return uint64_t(t.sec)*1000000000ULL+t.nanosec;}
class Publisher {
 std::function<void(const void*,std::type_index)> send;
 std::function<size_t()> count;
public:
 Publisher()=default;
 template<class M>explicit Publisher(std::shared_ptr<rclcpp::Publisher<M>> p){
  auto holder=std::make_shared<std::shared_ptr<rclcpp::Publisher<M>>>(p);
  release_entities.push_back([holder](){holder->reset();});
  send=[holder](const void* v,std::type_index type){
   if(type!=std::type_index(typeid(M))||!*holder)throw std::runtime_error("publisher unavailable or message type mismatch");
   auto begin=ltaom_budget::Clock::now();
   (*holder)->publish(*static_cast<const M*>(v));
   ++ltaom_budget::publish_calls;ltaom_budget::max_publish_ms=std::max(ltaom_budget::max_publish_ms,ltaom_budget::milliseconds(ltaom_budget::Clock::now()-begin));
  };count=[holder](){return *holder?(*holder)->get_subscription_count():size_t(0);};
 }
 template<class M>void publish(const M& m)const{if(!send)throw std::runtime_error("uninitialized publisher");send(&m,typeid(M));}
 template<class M>void publish(const std::shared_ptr<M>& m)const{publish(*m);}
 size_t getNumSubscribers()const{return count?count():0;}
};
template<class T>struct EntityHandle {
 std::shared_ptr<std::shared_ptr<T>> holder;
 EntityHandle()=default;
 template<class U>explicit EntityHandle(std::shared_ptr<U> value){
  holder=std::make_shared<std::shared_ptr<T>>(std::static_pointer_cast<T>(value));
  release_entities.push_back([p=holder](){p->reset();});
 }
};
using Subscriber=EntityHandle<rclcpp::SubscriptionBase>;
using ServiceServer=EntityHandle<rclcpp::Service<std_srvs::srv::Trigger>>;
struct NodeHandle {
 template<class T>void setParam(const std::string& name,const T& value){
  auto key=parameterName(name);if(!node->has_parameter(key))node->declare_parameter(key,value);
  else if(!node->set_parameter(rclcpp::Parameter(key,value)).successful)throw std::runtime_error("parameter rejected");
 }

 template<class T>void param(const std::string& name,T& value,const T& fallback){
  const auto key=parameterName(name);
  if(!node->has_parameter(key)){
   using P=std::conditional_t<std::is_same_v<T,float>,double,std::conditional_t<std::is_same_v<T,int>,int64_t,T>>;
   node->declare_parameter<P>(key,static_cast<P>(fallback));
  }
  getParam(name,value);
 }
 template<class T>bool getParam(const std::string& name,T& value){return ltaom_ros2::getParam(name,value);}
 template<class M>Publisher advertise(const std::string& topic,size_t depth){
  auto pub=node->create_publisher<M>(topic,rclcpp::QoS(std::min<size_t>(depth,256)).reliable());
  const auto q=pub->get_actual_qos().get_rmw_qos_profile();
  ltaom_budget::actual_qos["pub:"+topic]="{\"depth\":"+std::to_string(q.depth)+",\"reliability\":"+std::to_string(q.reliability)+",\"history\":"+std::to_string(q.history)+"}";
  Publisher result;return makePublisher<M>(pub);
 }
 template<class M>static Publisher makePublisher(typename rclcpp::Publisher<M>::SharedPtr pub){return Publisher(pub);}
 template<class M>Subscriber subscribe(const std::string& topic,size_t depth,void(*callback)(const std::shared_ptr<const M>&)){
  const size_t cap=std::is_same_v<M,sensor_msgs::msg::Imu>?8192:256;
  rclcpp::SubscriptionOptions options;
  options.event_callbacks.message_lost_callback=[topic](rclcpp::QOSMessageLostInfo& info){if(info.total_count_change>0)ltaom_budget::lost_messages[topic]+=info.total_count_change;};
  auto sub=node->create_subscription<M>(topic,rclcpp::QoS(std::min(depth,cap)).reliable(),callback,options);
  const auto q=sub->get_actual_qos().get_rmw_qos_profile();
  ltaom_budget::actual_qos["sub:"+topic]="{\"depth\":"+std::to_string(q.depth)+",\"reliability\":"+std::to_string(q.reliability)+",\"history\":"+std::to_string(q.history)+"}";
  return Subscriber(sub);
 }
 ServiceServer advertiseService(const std::string& name,bool(*callback)(std_srvs::Trigger::Request&,std_srvs::Trigger::Response&)){
  return ServiceServer(node->create_service<std_srvs::srv::Trigger>(name,[callback](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,std::shared_ptr<std_srvs::srv::Trigger::Response> res){callback(*req,*res);}));
 }
};
}
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("ltaom_frontend"),__VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("ltaom_frontend"),__VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("ltaom_frontend"),__VA_ARGS__)
#define ROS_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(rclcpp::get_logger("ltaom_frontend"),x)
#define ROS_ERROR_STREAM_THROTTLE(period,x) RCLCPP_ERROR_STREAM_THROTTLE(rclcpp::get_logger("ltaom_frontend"),*ltaom_ros2::node->get_clock(),static_cast<int64_t>((period)*1000),x)
#define ROS_ASSERT(x) do {if(!(x))throw std::runtime_error("algorithm assertion failed: " #x);} while(false)
