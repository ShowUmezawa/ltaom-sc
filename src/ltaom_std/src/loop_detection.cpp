#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <ltaom_transport/ros2_transport.hpp>
#include <ltaom_transport/submap_hash.hpp>
/*This code is the implementation of our paper "LTA-OM: Long-Term Association
LiDAR-Inertial Odometry and Mapping".

Current Developer: Zuhao Zou < zuhaozou@yahoo.com >

If you use any code of this repo in your academic research, please cite at least
one of our papers:
[1] Zou, Zuhao, et al. "LTA-OM: Long-Term Association LiDAR-Inertial
    Odometry and Mapping"
[2] Yuan, Chongjian, et al. "Std: Stable triangle descriptor for 3d place recognition"
[3] Xu, Wei, et al. "Fast-lio2: Fast direct lidar-inertial odometry."
[4] Xu, Wei, and Fu Zhang. "Fast-lio: A fast, robust lidar-inertial odometry
    package by tightly-coupled iterated kalman filter."
[5] Cai, Yixi, Wei Xu, and Fu Zhang. "ikd-Tree: An Incremental KD Tree for
    Robotic Applications."
[6] Lin, Jiarong, and Fu Zhang. "Loam-livox: A fast, robust, high-precision
    LiDAR odometry and mapping package for LiDARs of small FoV."

For commercial use, please contact me < zuhaozou@yahoo.com > and
Dr. Fu Zhang < fuzhang@hku.hk >.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/

#include "include/std.h"
#include "include/std_ba.h"
#include "include/candidate_export.hpp"

#include <condition_variable>
#include <mutex>
#include <pcl_conversions/pcl_conversions.h>
#include <deque>
#include <vector>
#include <string>

#include <thread>

#define for_pgo
// Standalone native ROS2 node; no nodelet ABI

bool flg_exit = false;

std::deque<ltaom_ros2::Time> time_buf;
ltaom_budget::Queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> lidar_buf("std_cloud",[](const pcl::PointCloud<pcl::PointXYZI>::Ptr& p){return sizeof(*p)+p->points.capacity()*sizeof(pcl::PointXYZI);});
std::map<uint64_t, nav_msgs::Odometry::ConstSharedPtr> odom_buf;
size_t odom_limit=1024,odom_peak=0,active_submap_limit=256LL*1024*1024,active_submap_bytes=0;

geometry_msgs::Pose current_pose;
geometry_msgs::Pose last_pose;
double position_threshold = 0.2;
double rotation_threshold = DEG2RAD(5);
bool is_sub_msg = false;

std::mutex mtx_buffer;
std::condition_variable sig_buffer;
int scan_count = 0;
double last_lidar_msgtime = 0;
std::fstream debug_file;//, result_file;

int multisession_mode = 0; //disabled by default
pcl::PointCloud<pcl::PointXYZ>::Ptr
    des_to_store(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    des_to_load(new pcl::PointCloud<pcl::PointXYZ>);
std::string save_directory;
bool export_candidates=false;
bool candidate_only=false;
bool std_retrieval=true;
bool flush_tail=false;
size_t consumed_scans=0;
bool std_drained=false;
bool stdDrain(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response) {
  int expected=-1;ltaom_ros2::param::get("std_expected_scans",expected);
  response.success=expected>=0 && scan_count==expected && consumed_scans==size_t(expected) && std_drained;
  response.message="received="+std::to_string(scan_count)+", consumed="+std::to_string(consumed_scans)+", expected="+std::to_string(expected);
  return true;
}
std::string candidate_session;
int credit_window=0;size_t published_submaps=0,completed_submaps=0;
void consumptionAck(const std_msgs::String::ConstSharedPtr& msg) {
  try {boost::property_tree::ptree v;std::istringstream stream(msg->data);boost::property_tree::read_json(stream,v);
    if(v.get<int>("schema")!=1 || v.get<std::string>("session_id")!=candidate_session)throw std::runtime_error("wrong session/schema");
    auto n=v.get<size_t>("completed_submaps");if(n<completed_submaps||n>published_submaps)throw std::runtime_error("invalid cumulative consumption ACK");completed_submaps=n;
  }catch(const std::exception& e){ltaom_budget::fail(std::string("STD consumption ACK: ")+e.what());}
}

ltaom_ros2::Publisher raw_candidate_pub;
std::ofstream raw_candidate_file;

//std::vector<Descriptor> descriptor_list_last;
pcl::PointCloud<pcl::PointXYZINormal>::Ptr corners_last_last_ (new pcl::PointCloud<pcl::PointXYZINormal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr corners_last_ (new pcl::PointCloud<pcl::PointXYZINormal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr corners_curr_ (new pcl::PointCloud<pcl::PointXYZINormal>);
bool stop_accumulation = false;
std::fstream time_lcd_file;

ltaom_ros2::Publisher pubRegisterCloud,pubCurrentCloud,pubCurrentBinary,
               pubMatchedCloud,pubMatchedBinary,pubSTD;

ltaom_ros2::Publisher submap_pose_pub, submap_cloud_body_pub, submap_id_pub,
               lc_tranform_pub, des_pub1, des_pub2, inter_triangle_pub,
               inter2_triangle_pub, lc_triangle_pub, lc_onprior_id_pub, line_pub;


void notificationCallback(const std_msgs::Float32MultiArray::ConstSharedPtr &msg)
{
  debug_file << "stop_accumulation = true" << std::endl;
  if (msg->data[0] == 1)
    stop_accumulation = true;
  else
    stop_accumulation = false;
}

uint64_t jump_time = 0;
void jumptimeCallback(const std_msgs::UInt64::ConstSharedPtr &msg)
{
  jump_time = msg->data;
}

bool VertexDisSort(std::tuple<STD,STD,float> a, std::tuple<STD,STD,float> b) { return (std::get<2>(a) < std::get<2>(b)); }

void pub_corners_pairs(ltaom_ros2::Publisher &handler3, const int frame_last, const int frame_curr,
                     const std::vector<std::pair<pcl::PointXYZINormal,pcl::PointXYZINormal>> &corners_pairs, const ltaom_ros2::Time& time_submap_local){
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr corners_cloud(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  if (corners_pairs.empty()){
    pcl::PointXYZINormal a_pt;
    a_pt.x = 0; a_pt.y = 0; a_pt.z = 0;
    corners_cloud->push_back(a_pt);
    corners_cloud->push_back(a_pt);
  }
  else{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr corners_last(
        new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr corners_curr(
        new pcl::PointCloud<pcl::PointXYZINormal>);
    for (auto a_pair:corners_pairs){
      corners_last->push_back(a_pair.first);
      corners_curr->push_back(a_pair.second);
    }
    *corners_cloud += *corners_curr;
    *corners_cloud += *corners_last;
  }

  corners_cloud->points[0].intensity = frame_last;
  corners_cloud->points[1].intensity = frame_curr;
  corners_cloud->height = 1;
  corners_cloud->width = corners_cloud->size();
  debug_file << frame_last << " " << frame_curr << " " << corners_cloud->size() << std::endl;
  sensor_msgs::PointCloud2 pub_cloud;
  pcl::toROSMsg(*corners_cloud, pub_cloud);
  pub_cloud.header.frame_id = "camera_init";
  pub_cloud.header.stamp = time_submap_local;
  handler3.publish(pub_cloud);

}

void pub_descriptors(ltaom_ros2::Publisher &handler1, ltaom_ros2::Publisher &handler2, ltaom_ros2::Publisher &handler3,
                     const std::vector<std::tuple<STD,STD,float>> &descriptor_pairs,
                     const ltaom_ros2::Time& time_submap_local, bool on_prior){
  if (descriptor_pairs.empty())
    return;
  visualization_msgs::Marker lines_;
  lines_.header.stamp = ltaom_ros2::Time::now();
  lines_.header.frame_id = "camera_init";
  lines_.ns = "association_lines";
  lines_.action = visualization_msgs::Marker::ADD;
  lines_.pose.orientation.w = 1.0f;
  lines_.id = 1;
  lines_.type = visualization_msgs::Marker::LINE_LIST;
  lines_.scale.x = 0.3;
  lines_.color.r = 0.0f;
  lines_.color.g = 1.0f;
  lines_.color.b = 0.0f;
  lines_.color.a = 1.0f;
  pcl::PointCloud<pcl::PointXYZI>::Ptr corners_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr corners_last(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr corners_curr(
        new pcl::PointCloud<pcl::PointXYZI>);
  geometry_msgs::Point point_a, point_b;
  pcl::PointXYZI pi;
  int count = 0;
  int k = 6;
  for (auto &a_pair:descriptor_pairs){
    count++;
    if (count > k) continue;
    point_a.x = std::get<0>(a_pair).binary_A_.location_[0];
    point_a.y = std::get<0>(a_pair).binary_A_.location_[1];
    point_a.z = std::get<0>(a_pair).binary_A_.location_[2];
    pi.x = point_a.x;
    pi.y = point_a.y;
    pi.z = point_a.z;
    corners_cloud->push_back(pi);
    corners_curr->push_back(pi);

    point_b.x = std::get<0>(a_pair).binary_B_.location_[0];
    point_b.y = std::get<0>(a_pair).binary_B_.location_[1];
    point_b.z = std::get<0>(a_pair).binary_B_.location_[2];

    lines_.points.push_back(point_a);
    lines_.points.push_back(point_b);

    point_a.x = std::get<0>(a_pair).binary_B_.location_[0];
    point_a.y = std::get<0>(a_pair).binary_B_.location_[1];
    point_a.z = std::get<0>(a_pair).binary_B_.location_[2];
    pi.x = point_a.x;
    pi.y = point_a.y;
    pi.z = point_a.z;
    corners_cloud->push_back(pi);
    corners_curr->push_back(pi);

    point_b.x = std::get<0>(a_pair).binary_C_.location_[0];
    point_b.y = std::get<0>(a_pair).binary_C_.location_[1];
    point_b.z = std::get<0>(a_pair).binary_C_.location_[2];

    lines_.points.push_back(point_a);
    lines_.points.push_back(point_b);

    point_a.x = std::get<0>(a_pair).binary_C_.location_[0];
    point_a.y = std::get<0>(a_pair).binary_C_.location_[1];
    point_a.z = std::get<0>(a_pair).binary_C_.location_[2];
    pi.x = point_a.x;
    pi.y = point_a.y;
    pi.z = point_a.z;
    corners_cloud->push_back(pi);
    corners_curr->push_back(pi);

    point_b.x = std::get<0>(a_pair).binary_A_.location_[0];
    point_b.y = std::get<0>(a_pair).binary_A_.location_[1];
    point_b.z = std::get<0>(a_pair).binary_A_.location_[2];

    lines_.points.push_back(point_a);
    lines_.points.push_back(point_b);
  }

  visualization_msgs::Marker lines_2;
  lines_2.header.stamp = ltaom_ros2::Time::now();
  lines_2.header.frame_id = "camera_init";
  lines_2.ns = "association_lines";
  lines_2.action = visualization_msgs::Marker::ADD;
  lines_2.pose.orientation.w = 1.0f;
  lines_2.id = 1;
  lines_2.type = visualization_msgs::Marker::LINE_LIST;
  lines_2.scale.x = 0.3;
  lines_2.color.r = 0.0f;
  lines_2.color.g = 0.0f;
  lines_2.color.b = 1.0f;
  lines_2.color.a = 1.0f;
  count = 0;
  for (auto &a_pair:descriptor_pairs){
    count++;
    if (count > k) continue;
    point_a.x = std::get<1>(a_pair).binary_A_.location_[0];
    point_a.y = std::get<1>(a_pair).binary_A_.location_[1];
    point_a.z = std::get<1>(a_pair).binary_A_.location_[2];
    pi.x = point_a.x;
    pi.y = point_a.y;
    pi.z = point_a.z;
    corners_cloud->push_back(pi);
    corners_last->push_back(pi);

    point_b.x = std::get<1>(a_pair).binary_B_.location_[0];
    point_b.y = std::get<1>(a_pair).binary_B_.location_[1];
    point_b.z = std::get<1>(a_pair).binary_B_.location_[2];

    lines_2.points.push_back(point_a);
    lines_2.points.push_back(point_b);

    point_a.x = std::get<1>(a_pair).binary_B_.location_[0];
    point_a.y = std::get<1>(a_pair).binary_B_.location_[1];
    point_a.z = std::get<1>(a_pair).binary_B_.location_[2];
    pi.x = point_a.x;
    pi.y = point_a.y;
    pi.z = point_a.z;
    corners_cloud->push_back(pi);
    corners_last->push_back(pi);

    point_b.x = std::get<1>(a_pair).binary_C_.location_[0];
    point_b.y = std::get<1>(a_pair).binary_C_.location_[1];
    point_b.z = std::get<1>(a_pair).binary_C_.location_[2];

    lines_2.points.push_back(point_a);
    lines_2.points.push_back(point_b);

    point_a.x = std::get<1>(a_pair).binary_C_.location_[0];
    point_a.y = std::get<1>(a_pair).binary_C_.location_[1];
    point_a.z = std::get<1>(a_pair).binary_C_.location_[2];
    pi.x = point_a.x;
    pi.y = point_a.y;
    pi.z = point_a.z;
    corners_cloud->push_back(pi);
    corners_last->push_back(pi);

    point_b.x = std::get<1>(a_pair).binary_A_.location_[0];
    point_b.y = std::get<1>(a_pair).binary_A_.location_[1];
    point_b.z = std::get<1>(a_pair).binary_A_.location_[2];

    lines_2.points.push_back(point_a);
    lines_2.points.push_back(point_b);
  }
  handler1.publish(lines_);
  handler2.publish(lines_2);
  corners_cloud->points[0].intensity = on_prior?-(std::get<1>(descriptor_pairs[0]).frame_number_+1):\
    std::get<1>(descriptor_pairs[0]).frame_number_;
  corners_cloud->points[1].intensity = std::get<0>(descriptor_pairs[0]).frame_number_;
  corners_cloud->height = 1;
  corners_cloud->width = corners_cloud->size();
  debug_file << corners_cloud->points[0].intensity << " " << corners_cloud->points[1].intensity << " " << corners_cloud->size() << std::endl;
  sensor_msgs::PointCloud2 pub_cloud;
  pcl::toROSMsg(*corners_cloud, pub_cloud);
  pub_cloud.header.frame_id = "camera_init";
  pub_cloud.header.stamp = time_submap_local;
  handler3.publish(pub_cloud);
}

void associate_consecutive_frames(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corners_curr, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corners_last,
                                  std::vector<std::pair<pcl::PointXYZINormal,pcl::PointXYZINormal>> &corners_pairs){

  if (corners_curr->empty() || corners_last->empty()) return;
  pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kd_tree(
      new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
  kd_tree->setInputCloud(corners_curr);

  std::vector<int> knn_indices;
  std::vector<float> knn_dis;
  const int k = 1;
  const float pt_dis_thr = 1.0f;
  const float dim_dis_thr = 0.6f;
  for (int r = 0; r < corners_last->size(); r++){
    pcl::PointXYZINormal a_pt = corners_last->points[r];
    kd_tree->nearestKSearch(a_pt, k, knn_indices, knn_dis);

    for (int idx = 0; idx < k; idx++){
      pcl::PointXYZINormal pt_nn = corners_curr->points[knn_indices[idx]];
      if (knn_dis[idx] > pt_dis_thr)
        continue;
      if (fabs(pt_nn.x - a_pt.x) > dim_dis_thr)
        continue;
      if (fabs(pt_nn.y - a_pt.y) > dim_dis_thr)
        continue;
      if (fabs(pt_nn.z - a_pt.z) > 0.3)
        continue;
      corners_pairs.push_back({pt_nn, a_pt});
      break;
    }
  }

}

void SigHandle(int sig) {
  flg_exit = true;
  ROS_WARN("catch sig %d", sig);
  sig_buffer.notify_all();
}

void pointCloudCallBack(const sensor_msgs::PointCloud2::ConstSharedPtr &msg) {
//  debug_file << "lidar msg time" << ltaom_ros2::nanoseconds(msg->header.stamp) << std::endl;
  std::unique_lock<std::mutex> buffer_lock(mtx_buffer);
  scan_count++;
  if (ltaom_ros2::seconds(msg->header.stamp) < last_lidar_msgtime) {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buf.clear();time_buf.clear();odom_buf.clear();
  }
  // PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  // p_pre->process(msg, ptr);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *ptr);
  lidar_buf.push_back(ptr);
  time_buf.push_back(msg->header.stamp);
  last_lidar_msgtime = ltaom_ros2::seconds(msg->header.stamp);
  buffer_lock.unlock();
  sig_buffer.notify_all();
}

void odomCallBack(const nav_msgs::Odometry::ConstSharedPtr &msg) {
//  debug_file << "odom msg time" << ltaom_ros2::nanoseconds(msg->header.stamp) << std::endl;
  auto key=ltaom_ros2::nanoseconds(msg->header.stamp);
  if(odom_buf.count(key))ltaom_budget::fail("duplicate STD odometry timestamp");
  if(odom_buf.size()>=odom_limit)ltaom_budget::fail("STD odometry queue capacity exceeded");
  odom_buf[key]=msg;odom_peak=std::max(odom_peak,odom_buf.size());
}

void load_prior_descriptor(std::unordered_map<STD_LOC, std::vector<STD>> &descriptor_map_prior,
                      std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &prior_plane_cloud_list){
  int prior_keyf_size = -1;
  std::string des_name = save_directory + "map_prior/descriptors.pcd";
  auto start1 = std::chrono::system_clock::now();
  pcl::io::loadPCDFile(des_name, *des_to_load);
  std::cout << "[LoadPrior]: loading prior desriptor pcd with size: " << des_to_load->size() << std::endl;
  load_descriptor(descriptor_map_prior, des_to_load, prior_keyf_size);
  std::cout << "[LoadPrior]: descriptor_prior size: " << descriptor_map_prior.size() << std::endl;
  auto end1 = std::chrono::system_clock::now();
  std::chrono::duration<double, std::milli> elapsed_ms1 = std::chrono::duration<double,std::milli>(end1 - start1);

  std::cout << "[LoadPrior]: prior_keyf_size: " << prior_keyf_size << std::endl;
  std::string pcd_name = save_directory+"map_prior/plane_all.pcd";
  auto start2 = std::chrono::system_clock::now();
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr prior_plane_cloud(new pcl::PointCloud<pcl::PointXYZINormal>) ;
  pcl::io::loadPCDFile(pcd_name, *prior_plane_cloud);

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr a_plane_cloud(new pcl::PointCloud<pcl::PointXYZINormal>) ;
  for (size_t i = 0; i < prior_plane_cloud->points.size(); i++ ){
    auto a_pt = prior_plane_cloud->points[i];
    if( fabs(a_pt.x - (-1010.1)) < 0.01){
      prior_plane_cloud_list.push_back(a_plane_cloud);
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZINormal>) ;
      a_plane_cloud = empty_cloud;
      continue;
    }
    a_plane_cloud -> push_back(a_pt);
  }
  std::cout << "[LoadPrior]: prior_plane_cloud_list size: " << prior_plane_cloud_list.size() << std::endl;

  auto end2 = std::chrono::system_clock::now();
  std::chrono::duration<double, std::milli> elapsed_ms2 = std::chrono::duration<double,std::milli>(end2 - start2);
}

bool detect_loopclosure( int & lc_his_id, std::vector<STD> &STD_list,
                         pcl::PointCloud<pcl::PointXYZINormal>::Ptr &frame_plane_cloud,
                         std::unordered_map<STD_LOC, std::vector<STD>> &STD_map,
                         std::vector<std::vector<BinaryDescriptor>> &history_binary_list,
                         std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &history_plane_list,
                         std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &key_cloud_list,
                         int &key_frame_id, ConfigSetting &config_setting, std::fstream &debug_file,
                         ltaom_ros2::Rate &loop, ltaom_ros2::Time &time_submap_local, bool on_prior)
{
    auto start2 = std::chrono::system_clock::now();
    auto t_candidate_search_begin = std::chrono::high_resolution_clock::now();
    std::vector<STDMatchList> alternative_match;
    if(std_retrieval || on_prior)candidate_searcher_old(config_setting, STD_map, STD_list,
                           alternative_match);
    if(export_candidates && !on_prior) {
        std_msgs::String m;m.data=stdCandidateBatch(candidate_session,key_frame_id,alternative_match,&config_setting);
        raw_candidate_pub.publish(m);raw_candidate_file<<m.data<<std::endl;
        if(!raw_candidate_file)throw std::runtime_error("STD candidate archive write failed");
    }
    if(candidate_only && !on_prior)return false;
    auto t_candidate_search_end = std::chrono::high_resolution_clock::now();

    // geometrical verification
    auto t_fine_loop_begin = std::chrono::high_resolution_clock::now();
    bool triggle_loop = false;
    Eigen::Vector3d best_t;
    Eigen::Matrix3d best_rot;
    Eigen::Vector3d loop_translation;
    Eigen::Matrix3d loop_rotation;
    std::vector<std::pair<STD, STD>> sucess_match_list;
    std::vector<std::pair<STD, STD>> unsucess_match_list;
    std::vector<std::pair<STD, STD>> sucess_match_list_publish;
    std::vector<std::pair<STD, STD>> unsucess_match_list_publish;
    int match_size = 0;
    int rough_size = 0;
    int candidate_id = -1;
    double mean_triangle_dis = 0;
    double mean_binary_similarity = 0;
    double outlier_mean_triangle_dis = 0;
    double outlier_mean_binary_similarity = 0;
    int match_frame = 0;
    double best_score = 0;
    double best_icp_score = 0;
    int best_frame = -1;
//    debug_file << "alternative_match.size(): "  << alternative_match.size() << std::endl;
    for (int i = 0; i < alternative_match.size(); i++) {
      if (alternative_match[i].match_list_.size() >= 4) {
        bool fine_sucess = false;
        Eigen::Matrix3d std_rot;
        Eigen::Vector3d std_t;
        debug_file << "[Rough match] rough match frame:"
                   << alternative_match[i].match_frame_ << " match size:"
                   << alternative_match[i].match_list_.size() << std::endl;
        sucess_match_list.clear();
        fine_loop_detection_tbb(
              config_setting, alternative_match[i].match_list_, fine_sucess,
              std_rot, std_t, sucess_match_list, unsucess_match_list, debug_file);
        if (fine_sucess) {
          double icp_score = geometric_verify(
                config_setting, frame_plane_cloud,
                history_plane_list[alternative_match[i].match_frame_], std_rot,
              std_t);
          double score = icp_score + sucess_match_list.size() * 1.0 / 1000;
          debug_file << "Fine sucess, Fine size:" << sucess_match_list.size()
                     << "  ,Icp score:" << icp_score << ", score:" << score
                     << std::endl;
          if (score > best_score) {
            unsucess_match_list_publish = unsucess_match_list;
            sucess_match_list_publish = sucess_match_list;
            best_frame = alternative_match[i].match_frame_;
            best_score = score;
            best_icp_score = icp_score;
            best_rot = std_rot;
            best_t = std_t;
            rough_size = alternative_match[i].match_list_.size();
            match_size = sucess_match_list.size();
            candidate_id = i;
          }
        }
      }
    }
    if (best_icp_score > config_setting.icp_threshold_) {
      loop_translation = best_t;
      loop_rotation = best_rot;
      match_frame = best_frame;
      triggle_loop = true;
      mean_triangle_dis = calc_triangle_dis(sucess_match_list_publish);
      mean_binary_similarity =
          calc_binary_similaity(sucess_match_list_publish);
      outlier_mean_triangle_dis =
          calc_triangle_dis(unsucess_match_list_publish);
      outlier_mean_triangle_dis =
          calc_binary_similaity(unsucess_match_list_publish);
      /* Disable icp as we have graph consistency check for SE3 check later.
         However, you better turn on for livox lidar or for the need of higher loop optimization accuracy. */
      //geometric_icp(frame_plane_cloud, history_plane_list[match_frame], loop_rotation, loop_translation);
      std::cout << "loop translation:" << loop_translation.transpose()
                << std::endl;
      std::cout << "loop rotation:" << std::endl
                << loop_rotation << std::endl;
    } else {
      triggle_loop = false;
    }
    auto t_fine_loop_end = std::chrono::high_resolution_clock::now();

    auto end2 = std::chrono::system_clock::now();
    std::chrono::duration<double, std::milli> elapsed_ms2 = std::chrono::duration<double,std::milli>(end2 - start2);
    if(triggle_loop)
      time_lcd_file << elapsed_ms2.count() << " " << 2 << std::endl;
    else
      time_lcd_file << elapsed_ms2.count() << " " << -2 << std::endl;

    if (triggle_loop) {
      debug_file << "[Loop Sucess] " << key_frame_id << "--" << match_frame
                 << ", candidate id:" << candidate_id
                 << ", icp:" << best_score << std::endl;
#ifdef for_pgo
      std::cout  << "[Loop Sucess] " << key_frame_id << "--" << match_frame
                 << ", candidate id:" << candidate_id
                 << ", icp:" << best_score << std::endl;

      std::vector<std::tuple<STD,STD,float>> descriptor_pairs;

      for (auto var : sucess_match_list_publish) {
        auto A_tran = loop_rotation * var.first.binary_A_.location_ + loop_translation;
        auto B_tran = loop_rotation * var.first.binary_B_.location_ + loop_translation;
        auto C_tran = loop_rotation * var.first.binary_C_.location_ + loop_translation;

        float ABC_dis = (var.second.binary_A_.location_ - A_tran).norm() +
            (var.second.binary_B_.location_ - B_tran).norm() + (var.second.binary_C_.location_ - C_tran).norm();
        descriptor_pairs.push_back({var.first, var.second, ABC_dis});
      }

      Eigen::Quaterniond q_opt(loop_rotation.cast<double>());
      geometry_msgs::PoseWithCovariance transform_msg;
      transform_msg.pose.position.x = loop_translation[0];
      transform_msg.pose.position.y = loop_translation[1];
      transform_msg.pose.position.z = loop_translation[2];
      transform_msg.pose.orientation.x = q_opt.x();
      transform_msg.pose.orientation.y = q_opt.y();
      transform_msg.pose.orientation.z = q_opt.z();
      transform_msg.pose.orientation.w = q_opt.w();
      transform_msg.covariance[0] = key_frame_id;
      transform_msg.covariance[1] = on_prior?-(match_frame+1):match_frame;
      lc_tranform_pub.publish(transform_msg);

      std::sort(descriptor_pairs.begin(), descriptor_pairs.end(), VertexDisSort);

      pub_descriptors(des_pub1, des_pub2, lc_triangle_pub, descriptor_pairs, time_submap_local, on_prior);
#endif
      debug_file << "[Loop Info] "
                 << "rough size:" << rough_size
                 << ", match size:" << match_size
                 << ", rough triangle dis:" << outlier_mean_triangle_dis
                 << ", fine triangle dis:" << mean_triangle_dis
                 << ", rough binary similarity:" << outlier_mean_triangle_dis
                 << ", fine binary similarity:" << mean_binary_similarity
                 << std::endl;
//      result_file << 1 << "," << match_frame << "," << candidate_id << ","
//                  << match_size << "," << rough_size << ","
//                  << loop_translation[0] << "," << loop_translation[1] << ","
//                  << loop_translation[2] << ",";

      if (!on_prior)
      {
        sensor_msgs::PointCloud2 pub_cloud;
        pcl::toROSMsg(*key_cloud_list[match_frame], pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedCloud.publish(pub_cloud);
        loop.sleep();
        pcl::PointCloud<pcl::PointXYZ> matched_key_points_cloud;
        for (auto var : history_binary_list[match_frame]) {
          pcl::PointXYZ pi;
          pi.x = var.location_[0];
          pi.y = var.location_[1];
          pi.z = var.location_[2];
          matched_key_points_cloud.push_back(pi);
        }
        pcl::toROSMsg(matched_key_points_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedBinary.publish(pub_cloud);
        Eigen::Vector3d color2(0, 1, 0);
        publish_binary(history_binary_list[match_frame], color2, "history",
                       pubSTD);
        loop.sleep();
      }
      debug_file << "1 ";
      for (size_t i = 0; i < sucess_match_list_publish.size(); i++) {
        sucess_match_list_publish[i].first.binary_A_.location_ =
            loop_rotation *
            sucess_match_list_publish[i].first.binary_A_.location_ +
            loop_translation;

        sucess_match_list_publish[i].first.binary_B_.location_ =
            loop_rotation *
            sucess_match_list_publish[i].first.binary_B_.location_ +
            loop_translation;
        sucess_match_list_publish[i].first.binary_C_.location_ =
            loop_rotation *
            sucess_match_list_publish[i].first.binary_C_.location_ +
            loop_translation;
      }
            debug_file << "2 ";
      publish_std(sucess_match_list_publish, pubSTD);
      nav_msgs::Odometry odom;
      odom.header.frame_id = "camera_init";
      odom.pose.pose.position.x = loop_translation[0];
      odom.pose.pose.position.y = loop_translation[1];
      odom.pose.pose.position.z = loop_translation[2];
      Eigen::Quaterniond loop_q(loop_rotation);
      odom.pose.pose.orientation.w = loop_q.w();
      odom.pose.pose.orientation.x = loop_q.x();
      odom.pose.pose.orientation.y = loop_q.y();
      odom.pose.pose.orientation.z = loop_q.z();
      // pubLocalizationInfo.publish(odom);
      loop.sleep();
            debug_file << "3 ";
      return true;
    } else {
      debug_file << "[Loop Fail] " << key_frame_id << ", icp:" << best_score
                 << std::endl;
      return false;
    }
}

#ifdef as_node
int mainLCFunction()
{
  int argc = 0; char** argv = nullptr;
  if (!ltaom_ros2::isInitialized()) ltaom_ros2::init(argc, argv, "ltaom_std");
  sleep(2); // make this node launch after LO node
  ltaom_ros2::NodeHandle nh;
  std::string config_file  = "config.yaml";
  if (nh.getParam("/lcd_config_path", config_file))
    std::cout << "Succeed in geting loop detection config file!" << std::endl;
  else
  {
    std::cerr << "Failed to get config file!" << std::endl;
    exit(-1);
  }
#else
int main(int argc, char **argv) {
  if (!ltaom_ros2::isInitialized()) ltaom_ros2::init(argc, argv, "ltaom_std");
  ltaom_ros2::NodeHandle nh;
  std::string config_file = "";
  if (nh.getParam("/lcd_config_path", config_file))
    std::cout << "Succeed in geting loop detection config file!" << std::endl;
  else
  {
    std::cerr << "Failed to get config file!" << std::endl;
    exit(-1);
  }
#endif
  // Nodelet manager workers must not concurrently mutate buffers consumed by
  // this loop. Own the subscription queue and dispatch on the processing thread.
  // The rclcpp single-threaded executor owns every callback and processing state.
  int expected_scans=-1;nh.param<int>("std_expected_scans",expected_scans,-1);
  auto drain_service=nh.advertiseService("/std/drain",stdDrain);
  nh.param<int>("multisession_mode",multisession_mode, 0);
  nh.param<std::string>("SaveDir",save_directory,"");
  nh.param<bool>("export_loop_candidates",export_candidates,false);
  nh.param<bool>("candidate_only",candidate_only,false);
  nh.param<bool>("std_retrieval_en",std_retrieval,true);
  nh.param<bool>("flush_tail_submap",flush_tail,false);
  if(multisession_mode!=0 || !candidate_only || !flush_tail)throw std::runtime_error("ROS2 migration requires single session, candidate_only, counted tail");
  if(save_directory.empty() || save_directory.back()!='/')throw std::runtime_error("SaveDir requires existing directory and trailing slash");
  ltaom_budget::output=save_directory+"queue_status.json";
  lidar_buf.configure(ltaom_ros2::node,256,1024LL*1024*1024);
  active_submap_limit=ltaom_budget::parameter(ltaom_ros2::node,"buffers.active_submap.max_bytes",256LL*1024*1024);
  odom_limit=ltaom_budget::parameter(ltaom_ros2::node,"buffers.std_odometry.max_count",1024);
  ltaom_budget::extra=[](){std::ostringstream o;o<<",\"odom_count\":"<<odom_buf.size()<<",\"odom_peak\":"<<odom_peak<<",\"active_submap_bytes\":"<<active_submap_bytes<<",\"active_submap_limit\":"<<active_submap_limit<<",\"consumed_scans\":"<<consumed_scans<<",\"published_submaps\":"<<published_submaps<<",\"completed_submaps\":"<<completed_submaps<<",\"credit_window\":"<<credit_window<<",\"inflight_submaps\":"<<(published_submaps-completed_submaps);return o.str();};
  ltaom_budget::write(true);
  nh.param<std::string>("session_id",candidate_session,"");
  if(candidate_session.empty() || candidate_session.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)throw std::runtime_error("invalid session_id");
  nh.param<int>("submap_credit_window",credit_window,0);
  if(credit_window<0)throw std::runtime_error("negative credit window");
  auto ack_sub=nh.subscribe("/loop_pipeline/consumption_ack",32,consumptionAck);
  auto submap_pub=nh.advertise<ltaom_interfaces::msg::Submap>("/loop_pipeline/submaps",256);
  if((candidate_only||!std_retrieval||flush_tail)&&multisession_mode!=0)throw std::runtime_error("D3 controls require single session");
  if(export_candidates) {
    ltaom_budget::output=save_directory+"queue_status.json";
  lidar_buf.configure(ltaom_ros2::node,256,1024LL*1024*1024);
  active_submap_limit=ltaom_budget::parameter(ltaom_ros2::node,"buffers.active_submap.max_bytes",256LL*1024*1024);
  odom_limit=ltaom_budget::parameter(ltaom_ros2::node,"buffers.std_odometry.max_count",1024);
  ltaom_budget::extra=[](){std::ostringstream o;o<<",\"odom_count\":"<<odom_buf.size()<<",\"odom_peak\":"<<odom_peak<<",\"active_submap_bytes\":"<<active_submap_bytes<<",\"active_submap_limit\":"<<active_submap_limit<<",\"consumed_scans\":"<<consumed_scans<<",\"published_submaps\":"<<published_submaps<<",\"completed_submaps\":"<<completed_submaps<<",\"credit_window\":"<<credit_window<<",\"inflight_submaps\":"<<(published_submaps-completed_submaps);return o.str();};
  ltaom_budget::write(true);
  nh.param<std::string>("session_id",candidate_session,"");
    if(multisession_mode!=0 || candidate_session.empty() || candidate_session.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
      throw std::runtime_error("candidate export requires single session and valid session_id");
    if(std::ifstream(save_directory+"std_candidates.jsonl"))throw std::runtime_error("refusing to overwrite STD candidates");
    raw_candidate_file.open(save_directory+"std_candidates.jsonl");
    if(!raw_candidate_file)throw std::runtime_error("cannot open STD candidates");
    raw_candidate_pub=nh.advertise<std_msgs::String>("/loop_pipeline/std_candidates",100);
  }

  ConfigSetting config_setting;
  load_config_setting(config_file, config_setting);
  std::cout << "waiting for point cloud data!" << std::endl;

  ltaom_ros2::Subscriber sub_cloud = nh.subscribe("/cloud_registered", 1000, pointCloudCallBack);
  ltaom_ros2::Subscriber sub_odom = nh.subscribe("/aft_mapped_to_init", 1000, odomCallBack);

  pubCurrentBinary = nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
  pubMatchedCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
  pubMatchedBinary = nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
  pubSTD = nh.advertise<visualization_msgs::MarkerArray>("/descriptor_line", 10);

  ltaom_ros2::Rate loop(50000);

  submap_pose_pub = nh.advertise<nav_msgs::Odometry>("/submap_pose", 100);
  pubCurrentCloud = nh.advertise<sensor_msgs::PointCloud2>("/clouds_submap", 100);
  submap_id_pub = nh.advertise<std_msgs::Int32>("/submap_ids", 100);
  lc_tranform_pub = nh.advertise<geometry_msgs::PoseWithCovariance>("/loop_closure_tranformation", 1);
  des_pub1 = nh.advertise<visualization_msgs::Marker>("/des_prev_submaps", 1);
  des_pub2 = nh.advertise<visualization_msgs::Marker>("/des_curr_submaps", 1);
  inter_triangle_pub = nh.advertise<sensor_msgs::PointCloud2>("/inter_triangles", 10);
  inter2_triangle_pub = nh.advertise<sensor_msgs::PointCloud2>("/inter2_triangles", 10);
  lc_triangle_pub = nh.advertise<sensor_msgs::PointCloud2>("/lc_triangles", 10);

  time_lcd_file = std::fstream(save_directory +"times_loopdetection_LTAOM.txt", std::fstream::out);
  time_lcd_file.precision(std::numeric_limits<double>::max_digits10);
  debug_file = std::fstream(save_directory + "lc_debug.txt", std::fstream::out);

  // save all point clouds of key frame
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_list;

  // save all planes of key frame
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> history_plane_list;

  // save all binary descriptors of key frame
  std::vector<std::vector<BinaryDescriptor>> history_binary_list;

  // save all STD descriptors of key frame
  std::vector<std::vector<STD>> history_STD_list;

  // save all poses(translation only) of key frame
  std::vector<Eigen::Vector3d> key_pose_list;

  // hash table, save all descriptor
  std::unordered_map<STD_LOC, std::vector<STD>> STD_map;

//  result_file = std::fstream(save_directory + "lc_result.txt", std::fstream::out);
  std::unordered_map<STD_LOC, std::vector<STD>> STD_map_prior;
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> prior_history_plane_list;
if (multisession_mode == 1){
  config_setting.skip_near_num_ = -100000;
  load_prior_descriptor(STD_map_prior, prior_history_plane_list);
}
  bool status = ltaom_ros2::ok();
  bool is_build_descriptor = false;
  int frame_number = 0;
  int key_frame_id = 0;
  ltaom_ros2::Time time_submap_local;
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_key_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  bool just_jump = false;
  bool std_eof_saved=false;
  std::vector<double> time_stored_buf;
  std::vector<uint64_t> stored_scan_ids;
  while (status) {
    ltaom_ros2::spinOnce();
    if(!ltaom_budget::fatal.empty())break;
    if(credit_window>0 && published_submaps-completed_submaps>=size_t(credit_window)) {std::this_thread::sleep_for(std::chrono::milliseconds(2));continue;}
    if (!lidar_buf.empty()) {
      // In streaming mode allocate once per submap, including stationary scans
      // and iterations waiting for matching odometry.
      if (frame_number == 0 && (!flush_tail || key_cloud_list.size()==size_t(key_frame_id))) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
            new pcl::PointCloud<pcl::PointXYZI>);
        key_cloud_list.push_back(temp_cloud);
        current_key_cloud->clear();
      }

      if (stop_accumulation){ //discard the seperated submap caused by jump
        if (!current_key_cloud->empty())
          current_key_cloud->clear();
        frame_number = 0;
        is_build_descriptor = false;
        corners_last_last_->clear();
        corners_last_->clear();
        just_jump = true;
        debug_file << "stop_accumulation" <<  std::endl;
        lidar_buf.pop_front();++consumed_scans;
        time_buf.pop_front();
        time_stored_buf.clear();stored_scan_ids.clear();
        continue;
      }
      time_submap_local = time_buf.front();
      if (just_jump){
        if(time_submap_local.toNSec() <= jump_time)
        {
          lidar_buf.pop_front();++consumed_scans;
          time_buf.pop_front();
          continue;
        }
      }
//      debug_file << "odom_buf.find(" << std::to_string(time_submap_local.toNSec()) << ")" << std::endl;
      if (odom_buf.empty()) continue;
      auto odom_buf_iter = odom_buf.find(time_submap_local.toNSec());
      if (odom_buf_iter == odom_buf.end())
      {
//        debug_file << "find fail" << std::endl;
        std::this_thread::sleep_for( std::chrono::milliseconds( 20 ) );
        std::this_thread::yield();
        continue;
      }
      auto odom_msg = odom_buf_iter->second;
      odom_buf.erase(odom_buf_iter); // Matched pose is consumed exactly once, not retained forever.

      current_pose = odom_msg->pose.pose;

      Eigen::Vector3d position_inc;
      position_inc << current_pose.position.x - last_pose.position.x,
          current_pose.position.y - last_pose.position.y,
          current_pose.position.z - last_pose.position.z;
      Eigen::Quaterniond current_quaternion(
            current_pose.orientation.w, current_pose.orientation.x,
            current_pose.orientation.y, current_pose.orientation.z);
      Eigen::Quaterniond last_quaternion(
            last_pose.orientation.w, last_pose.orientation.x, last_pose.orientation.y,
            last_pose.orientation.z);
      double rotation_inc = current_quaternion.angularDistance(last_quaternion);

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
          new pcl::PointCloud<pcl::PointXYZI>);
      cloud_ptr = lidar_buf.front();
      down_sampling_voxel(*cloud_ptr, config_setting.ds_size_);
      const size_t point_limit=active_submap_limit/sizeof(pcl::PointXYZI);
      if(cloud_ptr->size()>point_limit || current_key_cloud->size()>point_limit-cloud_ptr->size()) {
        ltaom_budget::fatal="STD active submap byte capacity exceeded";ltaom_budget::write(true);break;
      }
      const size_t needed=current_key_cloud->size()+cloud_ptr->size();
      if(needed>current_key_cloud->points.capacity())current_key_cloud->points.reserve(std::min(point_limit,std::max(needed,current_key_cloud->points.capacity()*2)));
      active_submap_bytes=current_key_cloud->points.capacity()*sizeof(pcl::PointXYZI);
      for (size_t i = 0; i < cloud_ptr->size(); i++) {
        current_key_cloud->points.push_back(cloud_ptr->points[i]);
      }
      assert(current_key_cloud->points.size() < 1e8); // Check if odom topic set correctly?
      //debug_file << "position_inc:" << position_inc.norm()
      //           << "  rotation_inc:" << RAD2DEG(rotation_inc) << std::endl;
      if(position_inc.norm() > 5){
        lidar_buf.pop_front();++consumed_scans;
        time_buf.pop_front();
        time_stored_buf.clear();stored_scan_ids.clear();
        last_pose = odom_msg->pose.pose;
        continue;
      }
      time_stored_buf.push_back(time_buf.front().toSec());stored_scan_ids.push_back(consumed_scans);

      if(position_inc.norm() < position_threshold && rotation_inc < rotation_threshold){
        lidar_buf.pop_front();++consumed_scans;
        time_buf.pop_front();
        continue;
      }
      if (frame_number < config_setting.sub_frame_num_ - 1) {
        frame_number++;
      } else {
        debug_file << "lidar_buf size: " << lidar_buf.size() <<" waiting to process" << std::endl;
        frame_number = 0;
        is_build_descriptor = true;
        just_jump = false;
//        down_sampling_voxel(*current_key_cloud, 0.5*config_setting.ds_size_);
      }
      last_pose = odom_msg->pose.pose;
      lidar_buf.pop_front();++consumed_scans;
      time_buf.pop_front();
    }

    // Explicit producer EOF: consumed count, empty scan queue, then partial tail.
    if(flush_tail) {
      int expected=-1;nh.getParam("/std_expected_scans",expected);
      if(expected>=0 && consumed_scans==size_t(expected) && lidar_buf.empty() && !is_build_descriptor) {
        if(!current_key_cloud->empty()&&!time_stored_buf.empty()) {is_build_descriptor=true;frame_number=0;}
        else if(!std_eof_saved) {
          std_eof_saved=true;std_drained=true;
          std::ofstream done(save_directory+"std_status.json.tmp");
          done<<"{\"drained\":true,\"consumed_scans\":"<<consumed_scans<<",\"submaps\":"<<key_frame_id<<"}"<<std::endl;done.close();
          std::rename((save_directory+"std_status.json.tmp").c_str(),(save_directory+"std_status.json").c_str());
        }
      }
    }
    // Core Part
    if (is_build_descriptor) {
      std::cout << std::endl;
      std::cout << "Key Frame:" << key_frame_id
                << ", cloud size:" << current_key_cloud->size() << std::endl;
      debug_file << std::endl;
      debug_file << "Key frame:" << key_frame_id
                 << ", cloud size:" << current_key_cloud->size() << std::endl;
      //std::string pcd_name = save_directory + "lcd_rawcloud" + std::to_string(key_frame_id) + ".pcd";
      //pcl::io::savePCDFileBinary(pcd_name, *current_key_cloud);
#ifdef for_pgo
      std_msgs::Int32 id_msg;
      id_msg.data = key_frame_id;
      submap_id_pub.publish(id_msg);
#endif

      auto start1 = std::chrono::system_clock::now();
      auto t1 = std::chrono::high_resolution_clock::now();
      auto t_build_descriptor_begin = std::chrono::high_resolution_clock::now();
      std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;
      init_voxel_map(config_setting, *current_key_cloud, voxel_map);
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr frame_plane_cloud(
          new pcl::PointCloud<pcl::PointXYZINormal>);
      get_plane(voxel_map, frame_plane_cloud);
      history_plane_list.push_back(frame_plane_cloud);
      std::vector<Plane *> proj_plane_list;
      std::vector<Plane *> merge_plane_list;
      get_project_plane(config_setting, voxel_map, proj_plane_list);
      debug_file << "proj_plane_list.size(): " << proj_plane_list.size() << std::endl;

      if (proj_plane_list.size() == 0) {
        Plane *single_plane = new Plane;
        single_plane->normal_ << 0, 0, 1;
        single_plane->center_ << current_key_cloud->points[0].x,
            current_key_cloud->points[0].y, current_key_cloud->points[0].z;
        merge_plane_list.push_back(single_plane);
      } else {
        sort(proj_plane_list.begin(), proj_plane_list.end(),
             plane_greater_sort);
        merge_plane(config_setting, proj_plane_list, merge_plane_list);
        sort(merge_plane_list.begin(), merge_plane_list.end(),
             plane_greater_sort);
      }
      debug_file << "merge_plane_list.size(): " << merge_plane_list.size() << std::endl;

      std::vector<BinaryDescriptor> binary_list;
      binary_extractor(config_setting, merge_plane_list, current_key_cloud,
                       binary_list);

      std::vector<STD> STD_list;
      generate_std(config_setting, binary_list, key_frame_id, STD_list);

      auto end1 = std::chrono::system_clock::now();
      auto elapsed_ms1 = std::chrono::duration<double,std::milli>(end1 - start1);
      debug_file << "elapsed_ms aft generate std: " << elapsed_ms1.count() << std::endl;

#ifdef for_pgo
      if (!corners_curr_->empty()) corners_curr_->clear();
      pcl::PointXYZINormal a_pt;
      for (auto a_bin:binary_list){
        a_pt.x = a_bin.location_[0]; a_pt.y = a_bin.location_[1]; a_pt.z = a_bin.location_[2];
        corners_curr_->push_back(a_pt);
      }

      std::vector<std::pair<pcl::PointXYZINormal,pcl::PointXYZINormal>> corners_pairs;
      debug_file << "[interT] corners sizes: " << corners_curr_->size() << " " << corners_last_->size() << std::endl;
      associate_consecutive_frames(corners_curr_, corners_last_, corners_pairs);
      pub_corners_pairs(inter_triangle_pub, key_frame_id-1, key_frame_id, corners_pairs, time_submap_local);
      corners_pairs.clear();
      if (key_frame_id > 1){
        debug_file << "[interT2] corners sizes: " << corners_curr_->size() << " " << corners_last_last_->size() << std::endl;
        associate_consecutive_frames(corners_curr_, corners_last_last_, corners_pairs);
        pub_corners_pairs(inter2_triangle_pub, key_frame_id-2, key_frame_id, corners_pairs, time_submap_local);
        *corners_last_last_ = *corners_last_;
      }
      *corners_last_ = *corners_curr_;

      nav_msgs::OdometryPtr odom_pub (new nav_msgs::Odometry());
      odom_pub->header.frame_id = "camera_init";
      odom_pub->child_frame_id = "before_pgo";
      odom_pub->header.stamp = time_submap_local;
      odom_pub->pose.pose.position = current_pose.position;
      odom_pub->pose.pose.orientation = current_pose.orientation;
      odom_pub->twist.covariance[0] = key_frame_id;
      odom_pub->twist.covariance[1] = time_stored_buf.front();
      odom_pub->twist.covariance[2] = time_stored_buf.back();
      ltaom_interfaces::msg::Submap packet;
      packet.schema=1;packet.session_id=candidate_session;packet.submap_id=key_frame_id;
      packet.first_scan_id=stored_scan_ids.front();packet.last_scan_id=stored_scan_ids.back();
      packet.association_time=time_submap_local;packet.anchor_pose=current_pose;
      time_stored_buf.clear();stored_scan_ids.clear();
      submap_pose_pub.publish(odom_pub);

      start1 = std::chrono::system_clock::now();
      history_binary_list.push_back(binary_list);
      debug_file << "[Corner] corner size:" << binary_list.size()
                 << "  descriptor size:" << STD_list.size() << std::endl;
      end1 = std::chrono::system_clock::now();
      elapsed_ms1 += std::chrono::duration<double,std::milli>(end1 - start1);
      time_lcd_file << elapsed_ms1.count() << " " << 3 << std::endl;
#endif

      //      history_binary_list.push_back(binary_list);
      sensor_msgs::PointCloud2 pub_cloud;
      pcl::toROSMsg(*current_key_cloud, pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pub_cloud.header.stamp = time_submap_local;
      pubCurrentCloud.publish(pub_cloud);
      packet.cloud=pub_cloud;packet.semantic_sha256=ltaom_protocol::submapHash(packet);submap_pub.publish(packet);++published_submaps;
      loop.sleep();

      pcl::PointCloud<pcl::PointXYZ> key_points_cloud;
      for (auto var : binary_list) {
        pcl::PointXYZ pi;
        pi.x = var.location_[0];
        pi.y = var.location_[1];
        pi.z = var.location_[2];
        key_points_cloud.push_back(pi);
      }
      pcl::toROSMsg(key_points_cloud, pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubCurrentBinary.publish(pub_cloud);
      loop.sleep();
      Eigen::Vector3d color1(1, 0, 0);
      publish_binary(binary_list, color1, "current", pubSTD);

      int lc_his_id;
      // candidate search
if (multisession_mode == 1)
{
      bool lc_onprior_found = detect_loopclosure(lc_his_id, STD_list, frame_plane_cloud, STD_map_prior,history_binary_list,
                                         prior_history_plane_list,key_cloud_list, key_frame_id,
                                         config_setting, debug_file, loop, time_submap_local, true);
      // Uncomment below to consider the case: no LC on prior map found, but LC on live map found
//      if (!lc_onprior_found && key_frame_id > 25)  detect_loopclosure(lc_his_id, descriptor_list, frame_plane_cloud,descriptor_map,
//                                                 plane_cloud_list,key_cloud_list, key_frame_id,
//                                                 config_setting, debug_file, rate, time_submap_local, false);
}else
{
      detect_loopclosure(lc_his_id, STD_list, frame_plane_cloud, STD_map,history_binary_list,
                                         history_plane_list,key_cloud_list, key_frame_id,
                                         config_setting, debug_file, loop, time_submap_local, false);
}
      is_build_descriptor = false;

      add_STD(STD_map, STD_list);

if (multisession_mode == 2)
{
      debug_file << "descriptor_map size: " << STD_map.size() << std::endl;
      save_descriptor(STD_list, des_to_store);
}
      down_sampling_voxel(*current_key_cloud, 0.5);
      for (size_t i = 0; i < current_key_cloud->size(); i++) {
        key_cloud_list.back()->push_back(current_key_cloud->points[i]);
      }
      key_frame_id++;
      if(flush_tail) {current_key_cloud->clear();time_stored_buf.clear();stored_scan_ids.clear();}
      std::ofstream progress(save_directory+"std_status.json.tmp");
      progress<<"{\"drained\":false,\"consumed_scans\":"<<consumed_scans<<",\"submaps\":"<<key_frame_id<<"}"<<std::endl;progress.close();
      std::rename((save_directory+"std_status.json.tmp").c_str(),(save_directory+"std_status.json").c_str());
      for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
        delete (iter->second);
      }
    }

if (multisession_mode == 2)
{
    bool save_one_time = false;
    nh.getParam("/save_prior_info", save_one_time);
    if(save_one_time){
      std::string des_name = save_directory + "map_prior/descriptors.pcd";
      pcl::io::savePCDFileBinary(des_name, *des_to_store);

      if (!history_plane_list.empty()){
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr plane_cloud_all (new pcl::PointCloud<pcl::PointXYZINormal>);
        for (int i = 0; i < history_plane_list.size(); i++){
          std::cout << history_plane_list[i]->size() << std::endl;
          *plane_cloud_all += *(history_plane_list[i]);
          pcl::PointXYZINormal pt_tmp;
          pt_tmp.x = -1010.1; pt_tmp.y = i; pt_tmp.z = 0;
          plane_cloud_all->push_back(pt_tmp);
        }
        std::string pcd_name = save_directory + "map_prior/plane_all.pcd";
        pcl::io::savePCDFileBinary(pcd_name, *plane_cloud_all);
      }
      nh.setParam("/save_prior_info", false);
      save_one_time = false;
    }
}
    status = ltaom_ros2::ok();
    if(lidar_buf.empty())std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ltaom_budget::write(true);
  ltaom_ros2::shutdown();
  return ltaom_budget::fatal.empty()?0:2;
}
