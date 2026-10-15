// Native ROS2 transport for the Stage4 single-session verified-Pose3 mode.
// Prior/odometry noise, update counts and trial checks follow the frozen ROS1
// single_session_backend.hpp / verified_pose3_backend.hpp. No 3E revocation.
#include <ltaom_transport/submap_hash.hpp>
#include <ltaom_transport/queue_budget.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <map>
#include <deque>
#include "factor_archive.hpp"
using Tree=factor_archive::Tree;
using Packet=ltaom_interfaces::msg::Submap;
using Pair=std::pair<int,int>;
using Cloud=pcl::PointCloud<pcl::PointXYZI>;
Tree parse(const std::string& text){Tree r;std::istringstream in(text);boost::property_tree::read_json(in,r);return r;}
gtsam::Pose3 pose(const geometry_msgs::msg::Pose& p){
 double n=p.orientation.x*p.orientation.x+p.orientation.y*p.orientation.y+p.orientation.z*p.orientation.z+p.orientation.w*p.orientation.w;
 if(!std::isfinite(n)||std::abs(n-1)>1e-8)throw std::runtime_error("invalid anchor quaternion");
 gtsam::Pose3 r(gtsam::Rot3::Quaternion(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z),gtsam::Point3(p.position.x,p.position.y,p.position.z));
 if(!r.matrix().allFinite())throw std::runtime_error("nonfinite anchor");return r;
}
std::string quoted(const std::string& s){
 std::ostringstream out;out<<'"';
 for(unsigned char c:s){if(c=='"'||c=='\\')out<<'\\'<<char(c);else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c);else out<<char(c);}
 out<<'"';return out.str();
}
class Backend:public rclcpp::Node {
 std::string root,session,fatal;size_t tx=0,accepted=0,rejected=0,received_loops=0,invalid=0;bool drained=false;
 size_t pending_bytes=0,peak_pending_bytes=0,peak_pending_count=0,packet_byte_limit=0;
 std::map<int,std::chrono::steady_clock::time_point> packet_arrivals;
 std::map<int,Packet::ConstSharedPtr> packets;std::map<int,Tree> records;std::map<Pair,Tree> waiting;
 std::map<Pair,std::string> payloads,terminal;std::map<size_t,Pair> loop_owners;std::deque<std::string> incoming;
 std::vector<gtsam::Pose3> raw;std::vector<Cloud::Ptr> clouds;std::vector<double> times;
 std::chrono::steady_clock::time_point last_status{};
 gtsam::ISAM2 solver;std::ofstream archive,ledger,bindings,verified;
 rclcpp::Subscription<Packet>::SharedPtr packets_sub;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr records_sub,loops_sub;
 rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr drain_srv;rclcpp::TimerBase::SharedPtr timer;
 static gtsam::ISAM2Params params(){gtsam::ISAM2Params p;p.relinearizeThreshold=.1;p.relinearizeSkip=1;p.enableDetailedResults=true;p.findUnusedFactorSlots=false;return p;}
 void guard(const std::function<void()>& fn){if(!fatal.empty())return;auto before=std::make_tuple(tx,received_loops,drained);try{fn();}catch(const std::exception& e){fatal=e.what();RCLCPP_ERROR(get_logger(),"backend stopped: %s",fatal.c_str());}if(!fatal.empty()||before!=std::make_tuple(tx,received_loops,drained)||std::chrono::steady_clock::now()-last_status>std::chrono::milliseconds(200))status();}
 void event(Pair p,const std::string& state,const std::string& reason,const gtsam::FactorIndices& indices={}){
  ledger<<"{\"transaction\":"<<tx<<",\"previous\":"<<p.first<<",\"current\":"<<p.second<<",\"status\":"<<quoted(state)<<",\"reason\":"<<quoted(reason)<<",\"factor_indices\":[";
  for(size_t i=0;i<indices.size();++i)ledger<<(i?",":"")<<indices[i];ledger<<"]}"<<std::endl;
  if(!ledger)throw std::runtime_error("ledger write failed");
 }
 void commit(const std::string& record,const gtsam::ISAM2Result& update){
  archive<<record.substr(0,record.size()-1)<<",\"factor_indices\":[";
  for(size_t i=0;i<update.newFactorsIndices.size();++i)archive<<(i?",":"")<<update.newFactorsIndices[i];archive<<"]}"<<std::endl;
  if(!archive)throw std::runtime_error("archive write failed");
 }
 void status(){
  last_status=std::chrono::steady_clock::now();
  std::ofstream f(root+"/backend_status.json.tmp");
  f<<"{\"schema\":1,\"fatal\":"<<quoted(fatal)<<",\"submaps\":"<<raw.size()<<",\"accepted\":"<<accepted<<",\"rejected\":"<<rejected<<",\"invalid_verified_loops\":"<<invalid<<",\"received_verified_loops\":"<<received_loops<<",\"pending_packets\":"<<packets.size()<<",\"pending_loops\":"<<waiting.size()+incoming.size()<<",\"drained\":"<<(drained?"true":"false")<<",\"terminal\":[";
  bool first=true;for(const auto& t:terminal){f<<(first?"":",")<<"{\"historical_id\":"<<t.first.first<<",\"current_id\":"<<t.first.second<<",\"status\":"<<quoted(t.second)<<"}";first=false;}f<<"]}\n";f.close();
  std::filesystem::rename(root+"/backend_status.json.tmp",root+"/backend_status.json");
  ltaom_budget::fatal=fatal;ltaom_budget::write(true);
 }
 void addSubmap(){
  int id=raw.size();if(!packets.count(id)||!records.count(id))return;
  const auto& packet=*packets.at(id);const auto& r=records.at(id);
  if(packet.schema!=1||packet.session_id!=session||packet.submap_id!=size_t(id)||packet.first_scan_id>packet.last_scan_id||packet.cloud.header.stamp!=packet.association_time)throw std::runtime_error("invalid submap identity");
  auto hash=ltaom_protocol::submapHash(packet);
  if(hash!=packet.semantic_sha256||hash!=r.get<std::string>("transport_semantic_sha256"))throw std::runtime_error("immutable submap hash mismatch");
  auto current=pose(packet.anchor_pose),record_pose=factor_archive::readPose(r.get_child("raw_anchor_pose"));
  if(!current.equals(record_pose,1e-12)||r.get<uint64_t>("first_scan_id")!=packet.first_scan_id||r.get<uint64_t>("last_scan_id")!=packet.last_scan_id)throw std::runtime_error("submap record payload mismatch");
  auto stamp=int64_t(packet.association_time.sec)*1000000000+packet.association_time.nanosec;
  if(r.get<int64_t>("association_time_ns")!=stamp)throw std::runtime_error("submap association mismatch");
  double time=r.get<double>("state_time");if(!std::isfinite(time)||(!times.empty()&&time<=times.back()))throw std::runtime_error("nonmonotonic state time");
  Cloud::Ptr cloud(new Cloud);pcl::fromROSMsg(packet.cloud,*cloud);for(const auto& p:*cloud)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(p.intensity))throw std::runtime_error("nonfinite point");
  gtsam::NonlinearFactorGraph graph;gtsam::Values initial;initial.insert(id,current);
  if(id==0)graph.add(gtsam::PriorFactor<gtsam::Pose3>(0,current,gtsam::noiseModel::Isotropic::Sigma(6,1e-6)));
  else {gtsam::Vector6 sig;sig<<.001,.001,.001,.01,.01,.01;graph.add(gtsam::BetweenFactor<gtsam::Pose3>(id-1,id,raw.back().between(current),gtsam::noiseModel::Diagonal::Sigmas(sig)));}
  ++tx;auto serialized=factor_archive::transaction(tx,session,graph,initial,1);solver.backup();gtsam::ISAM2Result update;
  try{update=solver.update(graph,initial);solver.update();}catch(...){solver.recover();throw;}
  commit(serialized,update);raw.push_back(current);clouds.push_back(cloud);times.push_back(time);pending_bytes-=packet.cloud.data.capacity();packet_arrivals.erase(id);packets.erase(id);
  bindings<<"{\"submap_id\":"<<id<<",\"semantic_sha256\":"<<quoted(hash)<<",\"cloud_sha256\":"<<quoted(r.get<std::string>("cloud_sha256"))<<"}"<<std::endl;
  if(!bindings)throw std::runtime_error("bindings write failed");publishPath();
 }
 void ingestLoops(){
  while(!incoming.empty()){
   auto text=incoming.front();incoming.pop_front();++received_loops;Pair pair{-1,-1};
   try{
    auto r=parse(text);pair={r.get<int>("historical_id"),r.get<int>("current_id")};
    if(r.get<int>("schema")!=1||r.get<std::string>("session_id")!=session||r.get<int>("revision")!=0||pair.first<0||pair.first>=pair.second||size_t(pair.second)>=raw.size())throw std::runtime_error("invalid verified identity");
    if(r.get<std::string>("loop_id")!=session+":loop:"+std::to_string(pair.first)+":"+std::to_string(pair.second)||r.get<std::string>("state")!="verified"||r.get<std::string>("measurement_convention")!="T_Bh_Bc"||r.get<std::string>("validator_version")!="d2-v1")throw std::runtime_error("invalid verified contract");
    factor_archive::readPose(r.get_child("measurement"));auto sig=factor_archive::numbers(r.get_child("sigmas"));
    if(sig.size()!=6||r.get<std::string>("sigma_order")!="rx_ry_rz_tx_ty_tz"||r.get_child("sources").empty()||r.get_child("quality").empty())throw std::runtime_error("invalid verification metadata");
    for(int i=0;i<6;++i)if(sig[i]<(i<3?.005:.03)||sig[i]>(i<3?.2:1.))throw std::runtime_error("noise_out_of_bounds");
    for(auto v:{std::make_pair(pair.first,"historical_cloud_sha256"),std::make_pair(pair.second,"current_cloud_sha256")})if(r.get<std::string>(v.second)!=records.at(v.first).get<std::string>("cloud_sha256"))throw std::runtime_error("submap_hash_mismatch");
    if(payloads.count(pair)){event(pair,payloads.at(pair)==text?"duplicate":"conflict","measurement_replacement_requires_E1");continue;}
    payloads.emplace(pair,text);waiting.emplace(pair,r);verified<<text<<std::endl;if(!verified)throw std::runtime_error("verified write failed");
   }catch(const std::exception& e){if(std::string(e.what())=="verified write failed")throw;++invalid;event(pair,"invalid",e.what());}
  }
 }
 void processLoop(){
  if(waiting.empty())return;
  auto chosen=waiting.begin();for(auto it=waiting.begin();it!=waiting.end();++it)if(std::make_pair(it->first.second,it->first.first)<std::make_pair(chosen->first.second,chosen->first.first))chosen=it;
  auto pair=chosen->first;auto r=chosen->second;waiting.erase(chosen);auto sigmas=factor_archive::numbers(r.get_child("sigmas"));gtsam::Vector6 sig;for(int i=0;i<6;++i)sig[i]=sigmas[i];
  auto noise=gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(1),gtsam::noiseModel::Diagonal::Sigmas(sig));
  gtsam::NonlinearFactorGraph graph;graph.add(gtsam::BetweenFactor<gtsam::Pose3>(pair.first,pair.second,factor_archive::readPose(r.get_child("measurement")),noise));
  ++tx;event(pair,"trial","verified_pose3");auto serialized=factor_archive::transaction(tx,session,graph,gtsam::Values(),4,{r.get<std::string>("loop_id")});
  solver.backup();gtsam::ISAM2Result update;std::string reason;double max_loop=0,max_graph=0;
  try{
   update=solver.update(graph,gtsam::Values());for(int i=0;i<4;++i)solver.update();auto values=solver.calculateEstimate();
   for(auto k:values.keys())if(!values.at<gtsam::Pose3>(k).matrix().allFinite())throw std::runtime_error("nonfinite_pose");
   const auto& factors=solver.getFactorsUnsafe();
   for(size_t i=0;i<factors.size();++i){auto f=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factors[i]);if(!f)continue;auto model=f->noiseModel();if(auto robust=boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(model))model=robust->noise();
    double residual=model->whiten(f->unwhitenedError(values)).norm()/std::sqrt(double(f->dim()));double weighted=std::sqrt(2*f->error(values)/f->dim());
    if(!std::isfinite(residual)||!std::isfinite(weighted))throw std::runtime_error("nonfinite_normalized_residual");max_graph=std::max(max_graph,residual);if(loop_owners.count(i)||i==update.newFactorsIndices.at(0))max_loop=std::max(max_loop,residual);
   }
   if(max_loop>get_parameter("max_loop_normalized_residual").as_double())reason="loop_normalized_residual";
   else if(max_graph>get_parameter("max_graph_normalized_residual").as_double())reason="graph_normalized_residual";
  }catch(const std::exception& e){reason=std::string("solver_exception: ")+e.what();}
  if(!reason.empty()){solver.recover();terminal[pair]="rejected";++rejected;event(pair,"rejected",reason);return;}
  commit(serialized,update);loop_owners[update.newFactorsIndices.at(0)]=pair;terminal[pair]="accepted";++accepted;event(pair,"accepted","pose3_committed",update.newFactorsIndices);publishPath();
 }
 void publishPath(){
  auto values=solver.calculateEstimate();nav_msgs::msg::Path path;path.header.frame_id="camera_init";
  for(size_t i=0;i<raw.size();++i){geometry_msgs::msg::PoseStamped p;p.header.frame_id=path.header.frame_id;p.header.stamp=rclcpp::Time(int64_t(std::llround(times[i]*1e9)));const auto& v=values.at<gtsam::Pose3>(i);auto q=v.rotation().toQuaternion();p.pose.position.x=v.x();p.pose.position.y=v.y();p.pose.position.z=v.z();p.pose.orientation.x=q.x();p.pose.orientation.y=q.y();p.pose.orientation.z=q.z();p.pose.orientation.w=q.w();path.poses.push_back(p);}
  if(!path.poses.empty())path.header.stamp=path.poses.back().header.stamp;path_pub->publish(path);
 }
 void exportResult(){
  auto values=solver.calculateEstimate();std::ofstream poses(root+"/optimized_poses.txt");poses<<std::setprecision(17);Cloud map;size_t stride=get_parameter("map_stride").as_int();
  for(size_t i=0;i<raw.size();++i){const auto& v=values.at<gtsam::Pose3>(i);auto q=v.rotation().toQuaternion();poses<<i<<' '<<v.x()<<' '<<v.y()<<' '<<v.z()<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<'\n';Eigen::Matrix4d transform=v.matrix()*raw[i].inverse().matrix();Cloud source,aligned;for(size_t j=0;j<clouds[i]->size();j+=stride)source.push_back(clouds[i]->at(j));pcl::transformPointCloud(source,aligned,transform);map+=aligned;}
  poses.flush();if(!poses||pcl::io::savePCDFileBinary(root+"/cloud_result.pcd",map)!=0)throw std::runtime_error("result export failed");
 }
public:
 Backend():Node("ltaom_backend"),solver(params()){
  root=declare_parameter<std::string>("output_dir","");session=declare_parameter<std::string>("session_id","");
  declare_parameter<int>("expected_submaps",-1);declare_parameter<int>("expected_verified_loops",-1);declare_parameter<int>("map_stride",1);
  declare_parameter<double>("max_loop_normalized_residual",8.);declare_parameter<double>("max_graph_normalized_residual",20.);
  if(root.empty()||session.empty()||session.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos||get_parameter("map_stride").as_int()<1)throw std::runtime_error("invalid backend parameters");
  for(auto name:{"max_loop_normalized_residual","max_graph_normalized_residual"})if(!(get_parameter(name).as_double()>0&&std::isfinite(get_parameter(name).as_double())))throw std::runtime_error("invalid residual limit");
  if(std::filesystem::exists(root))throw std::runtime_error("refusing to overwrite output directory");std::filesystem::create_directories(root);
  archive.open(root+"/factor_transactions.jsonl");ledger.open(root+"/loop_ledger.jsonl");bindings.open(root+"/submap_bindings.jsonl");verified.open(root+"/verified_inputs.jsonl");
  packet_byte_limit=declare_parameter<int64_t>("buffers.pending_submap.max_bytes",1024LL*1024*1024);
  if(packet_byte_limit==0 || packet_byte_limit>size_t(INT64_MAX))throw std::runtime_error("invalid pending submap byte limit");
  ltaom_budget::output=root+"/queue_status.json";
  ltaom_budget::queues["backend_submap"]=[this](){std::ostringstream o;double oldest=0;for(const auto& t:packet_arrivals)oldest=std::max(oldest,ltaom_budget::milliseconds(ltaom_budget::Clock::now()-t.second));o<<"{\"count\":"<<packets.size()<<",\"bytes\":"<<pending_bytes<<",\"peak_count\":"<<peak_pending_count<<",\"peak_bytes\":"<<peak_pending_bytes<<",\"max_count\":512,\"max_bytes\":"<<packet_byte_limit<<",\"oldest_wait_ms\":"<<oldest<<",\"high\":"<<((packets.size()>=384||pending_bytes>=.75*packet_byte_limit)?"true":"false")<<"}";return o.str();};
  path_pub=create_publisher<nav_msgs::msg::Path>("/aft_pgo_path",rclcpp::QoS(10).reliable());
  packets_sub=create_subscription<Packet>("/loop_pipeline/submaps",rclcpp::QoS(256).reliable(),[this](Packet::ConstSharedPtr m){guard([&]{if(drained||m->session_id!=session||m->submap_id>raw.size()+512||m->submap_id<raw.size()||packets.count(m->submap_id)||packets.size()>=512)throw std::runtime_error("invalid/duplicate/overflow submap packet");if(m->cloud.data.capacity()>packet_byte_limit-pending_bytes)throw std::runtime_error("backend submap byte capacity exceeded");packets.emplace(m->submap_id,m);packet_arrivals[m->submap_id]=ltaom_budget::Clock::now();pending_bytes+=m->cloud.data.capacity();peak_pending_bytes=std::max(peak_pending_bytes,pending_bytes);peak_pending_count=std::max(peak_pending_count,packets.size());});});
  records_sub=create_subscription<std_msgs::msg::String>("/loop_pipeline/submap_records",rclcpp::QoS(256).reliable(),[this](std_msgs::msg::String::ConstSharedPtr m){guard([&]{auto r=parse(m->data);int id=r.get<int>("submap_id");if(drained||id!=int(records.size())||r.get<int>("schema")!=1||r.get<std::string>("session_id")!=session||records.size()-raw.size()>=512)throw std::runtime_error("invalid record sequence/session/overflow");records.emplace(id,r);});});
  loops_sub=create_subscription<std_msgs::msg::String>("/loop_pipeline/verified_loops",rclcpp::QoS(256).reliable(),[this](std_msgs::msg::String::ConstSharedPtr m){guard([&]{if(drained||incoming.size()+waiting.size()>=512)throw std::runtime_error("input after EOF or loop queue overflow");incoming.push_back(m->data);});});
  timer=create_wall_timer(std::chrono::milliseconds(2),[this]{guard([&]{while(packets.count(raw.size())&&records.count(raw.size()))addSubmap();ingestLoops();while(!waiting.empty())processLoop();});});
  drain_srv=create_service<std_srvs::srv::Trigger>("/loop_optimizer/drain",[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){guard([&]{int ns=get_parameter("expected_submaps").as_int(),nl=get_parameter("expected_verified_loops").as_int();if(!drained&&ns>0&&nl>=0&&raw.size()==size_t(ns)&&records.size()==size_t(ns)&&received_loops==size_t(nl)&&packets.empty()&&incoming.empty()&&waiting.empty()){exportResult();drained=true;}});res->success=drained&&fatal.empty();res->message=fatal.empty()?(drained?"drained":"await expected counts"):fatal;});status();
 }
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{auto node=std::make_shared<Backend>();rclcpp::spin(node);node.reset();rclcpp::shutdown();return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}}
