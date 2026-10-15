#include "preprocess.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
void check(bool c, const char* what) { if(!c) throw std::runtime_error(what); }
sensor_msgs::PointCloud2::Ptr makeCloud(int lines) {
  pcl::PointCloud<hesai_ros::Point> p;
  for(int i=0;i<120;++i) {
    hesai_ros::Point q{}; q.x=3+i*.01; q.y=1; q.z=2; q.intensity=42;
    q.timestamp=1781091000.0 + (119-i)*.1/119.; q.ring=i%lines;
    p.push_back(q);
  }
  sensor_msgs::PointCloud2::Ptr m(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(p,*m); m->header.stamp.fromSec(1781091000.0);return m;
}
int main(int argc,char**argv) {
  ros::init(argc,argv,"hesai_adapter_test",ros::init_options::AnonymousName);
  ros::Time::init();
  try {
    for(int lines : {16,32}) {
      Preprocess p; p.set(false,HESAI,.5,3); p.N_SCANS=lines;
      PointCloudXYZI::Ptr out(new PointCloudXYZI);
      auto m=makeCloud(lines);p.process(m,out);
      check(p.last_scan_valid,"valid cloud rejected");check(out->size()==40,"unexpected point count");
      check(std::abs(p.scan_duration_s-.1)<1e-6,"absolute double timestamp lost precision");
      for(size_t i=1;i<out->size();++i)check((*out)[i].curvature>=(*out)[i-1].curvature,"time not ordered");
      check((*out)[0].intensity==42,"intensity lost");
      // Filtering removes the latest point in this ascending scan.
      pcl::PointCloud<hesai_ros::Point> ascending;
      pcl::fromROSMsg(*m,ascending);std::reverse(ascending.begin(),ascending.end());
      pcl::toROSMsg(ascending,*m);m->header.stamp.fromSec(1781091000.0);p.process(m,out);
      check(p.scan_duration_s > out->back().curvature/1000.+.001,"scan end changed by decimation");
      auto wrong=makeCloud(lines);for(auto &f:wrong->fields)if(f.name=="timestamp")f.datatype=7;
      p.process(wrong,out);check(!p.last_scan_valid && out->empty(),"FLOAT32 epoch accepted");
      wrong=makeCloud(lines);wrong->header.stamp.fromSec(1781090999.0);
      p.process(wrong,out);check(!p.last_scan_valid,"wrong header accepted");
      wrong=makeCloud(lines);wrong->data.pop_back();p.process(wrong,out);check(!p.last_scan_valid,"truncated cloud accepted");
      p.N_SCANS=lines-1;p.process(makeCloud(lines),out);check(!p.last_scan_valid,"out of range ring accepted");
      p.N_SCANS=lines; p.point_filter_num=0;p.process(makeCloud(lines),out);check(!p.last_scan_valid,"zero decimation accepted");
      p.point_filter_num=3;sensor_msgs::PointCloud2::Ptr empty(new sensor_msgs::PointCloud2);
      p.process(empty,out);check(!p.last_scan_valid && out->empty(),"empty cloud accepted");
    }
    std::cout<<"PASS Hesai16/32: precision, scan duration, sorting, intensity, field/layout/header/ring/empty guards\n";
  } catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
}
