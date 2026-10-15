// Transactional single-session scheduling. Included after the legacy helpers.
// The prior-map/multisession path remains in main_pgo().
#pragma once
#include <std_srvs/Trigger.h>
#include <map>
#include <set>
#include <cstdio>
#include "factor_archive.hpp"
#include <std_msgs/String.h>
#include <openssl/sha.h>
#include <ros/serialization.h>

namespace single_backend {
using Pair = std::pair<int,int>; // historical, current
using Cloud = pcl::PointCloud<PointType>::Ptr;
struct Candidate { Pair pair; gtsam::Pose3 relative; Cloud points; double overlap; };
struct State {
    std::map<int,nav_msgs::OdometryConstPtr> odoms;
    std::map<uint64_t,sensor_msgs::PointCloud2ConstPtr> clouds;
    std::map<int,Cloud> adjacent;
    std::map<Pair,Cloud> points;
    std::map<Pair,geometry_msgs::PoseWithCovarianceConstPtr> proposals;
    std::map<Pair,std::string> terminal;
    std::vector<Candidate> waiting;
    std::map<size_t,Pair> factor_owner;
    std::map<std::string,size_t> received;
    std::ofstream ledger, inventory;
    bool eof=false, drained=false, audit=false;
    std::string fatal;
    size_t transaction=0, accepted=0, rejected=0, replacements=0;
    int force_reject=-1;
    // Only the dedicated ablation executable reads these experimental controls.
    bool adjacent_enabled=true, loops_enabled=true, compression_enabled=true;
    std::string loop_policy="native";
    std::set<Pair> fixed_pairs;
    size_t skipped=0;
    std::ofstream timing;
    bool rebuildable=false;
    std::string session_id;
    std::ofstream archive;
    bool verified_mode=false;
    bool streaming_records=false;
    std::string record_path;
    std::streamoff record_offset=0;
    ros::Subscriber verified_subscriber;
    std::deque<std::string> verified_queue;
    std::map<Pair,factor_archive::Tree> verified;
    std::map<Pair,std::string> verified_payload;
    std::map<int,factor_archive::Tree> submap_records;
    std::ofstream verified_audit;
    double max_loop_normalized=8.,max_graph_normalized=20.;
} state;

void archiveCommit(const std::string& record,const gtsam::ISAM2Result& result) {
    if(!state.rebuildable)return;
    state.archive<<record.substr(0,record.size()-1)<<",\"factor_indices\":[";
    for(size_t i=0;i<result.newFactorsIndices.size();++i)state.archive<<(i?",":"")<<result.newFactorsIndices[i];
    state.archive<<"]}"<<std::endl;
    if(!state.archive)throw std::runtime_error("factor archive write failed; stop to avoid unaudited updates");
}

void recordTiming(const char* stage, const std::chrono::steady_clock::time_point& start) {
    if(state.timing) state.timing << state.transaction << ',' << stage << ',' << keyframes_.size() << ','
        << std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count() << std::endl;
}

std::string quote(const std::string& s) {
    std::string out="\"";
    for(char c:s) { if(c=='\\'||c=='\"') out+='\\'; if(c=='\n') out+="\\n"; else out+=c; }
    return out+'\"';
}
void event(Pair p,const std::string& status,const std::string& reason,
           const std::vector<size_t>& indices={},const std::vector<size_t>& removed={}) {
    auto& f=state.ledger;
    f << "{\"transaction\":" << state.transaction << ",\"previous\":" << p.first
      << ",\"current\":" << p.second << ",\"status\":" << quote(status)
      << ",\"loop_id\":" << quote(state.rebuildable?state.session_id+":loop:"+std::to_string(p.first)+":"+std::to_string(p.second):"")
      << ",\"revision\":0,\"graph_epoch\":0"
      << ",\"reason\":" << quote(reason) << ",\"factor_indices\":[";
    for(size_t i=0;i<indices.size();++i) f << (i?",":"") << indices[i];
    f << "],\"removed_indices\":[";
    for(size_t i=0;i<removed.size();++i) f << (i?",":"") << removed[i];
    f << "]}" << std::endl;
}
void dump(const std::string& stage) {
    if(!state.audit) return;
    const auto& graph=isam_->getFactorsUnsafe();
    for(size_t i=0;i<graph.size();++i) {
        const auto& f=graph[i]; if(!f) continue;
        std::string kind="other";
        if(boost::dynamic_pointer_cast<gtsam::TunningPointPairsFactor<gtsam::Pose3>>(f)) kind="point_pair";
        if(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(f)) kind="between";
        if(boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(f)) kind="prior";
        const auto& k=f->keys();
        state.inventory << state.transaction << ',' << stage << ',' << keyframes_.size() << ',' << i << ',' << kind << ','
                        << (k.empty()?-1:static_cast<long long>(k[0])) << ','
                        << (k.size()<2?-1:static_cast<long long>(k[1])) << '\n';
    }
    state.inventory.flush();
}
void clearStaging() {
    gts_graph_.resize(0); gts_graph_recover_.resize(0);
    gts_init_vals_.clear(); gts_init_vals_recover_.clear();
}
void refresh() { gts_cur_vals_=isam_->calculateEstimate(); correctPosesAndSaveTxt(); }
void status() {
    std::ofstream f(save_directory+"backend_status.json.tmp");
    f << "{\"input_closed\":" << (state.eof?"true":"false") << ",\"drained\":" << (state.drained?"true":"false")
      << ",\"fatal\":" << quote(state.fatal) << ",\"submaps\":" << keyframes_.size()
      << ",\"accepted\":" << state.accepted << ",\"rejected\":" << state.rejected
      << ",\"skipped\":" << state.skipped
      << ",\"rebuildable\":" << (state.rebuildable?"true":"false")
      << ",\"session_id\":" << quote(state.session_id) << ",\"graph_epoch\":0"
      << ",\"pending_confirmation\":" << state.waiting.size() << ",\"pending_proposals\":" << state.proposals.size()
      << ",\"registered_submaps\":" << state.submap_records.size()
      << ",\"streaming_records\":" << (state.streaming_records?"true":"false")
      << ",\"pending_verified\":" << state.verified.size()
      << ",\"pending_points\":" << state.points.size() << ",\"pending_odoms\":" << state.odoms.size()
      << ",\"pending_clouds\":" << state.clouds.size() << ",\"pending_adjacent\":" << state.adjacent.size()
      << ",\"received\":{";
    bool first=true; for(const auto& r:state.received) { f << (first?"":",") << quote(r.first) << ':' << r.second; first=false; }
    f << "}}\n"; f.close();
    std::rename((save_directory+"backend_status.json.tmp").c_str(),(save_directory+"backend_status.json").c_str());
}
Cloud decode(const sensor_msgs::PointCloud2ConstPtr& msg) {
    Cloud c(new pcl::PointCloud<PointType>); pcl::fromROSMsg(*msg,*c); return c;
}
Pair pointPair(const Cloud& c) {
    if(c->size()<2) throw std::runtime_error("point pair message has fewer than two points");
    const double a=c->points[0].intensity,b=c->points[1].intensity;
    if(!std::isfinite(a)||!std::isfinite(b)||a!=std::floor(a)||b!=std::floor(b)) throw std::runtime_error("invalid point pair IDs");
    return {int(a),int(b)};
}
void reject(Pair p,const std::string& why) {
    if(state.terminal.count(p)) return;
    state.terminal[p]="rejected"; ++state.rejected; event(p,"rejected",why);
    state.points.erase(p);
}
// Only this worker owns the maps and solver. Callbacks only append under mtx_sub_.
void loadStreamingRecords() {
    if(!state.streaming_records)return;
    std::ifstream file(state.record_path,std::ios::binary);if(!file)return;
    file.seekg(state.record_offset);std::string line;
    for(int count=0;count<64 && std::getline(file,line);++count) {
        if(file.eof())break; // Writer may not have committed the newline yet.
        factor_archive::Tree r;std::istringstream in(line);boost::property_tree::read_json(in,r);
        int id=r.get<int>("submap_id");
        if(state.eof||id!=int(state.submap_records.size())||r.get<std::string>("session_id")!=state.session_id||r.get<int>("schema")!=1)
            throw std::runtime_error("invalid_streaming_submap_identity");
        factor_archive::readPose(r.get_child("raw_anchor_pose"));
        for(auto key:{"cloud_sha256","cloud_message_sha256","odom_message_sha256"}) {
            auto hash=r.get<std::string>(key);
            if(hash.size()!=64||hash.find_first_not_of("0123456789abcdef")!=std::string::npos)throw std::runtime_error("invalid_streaming_hash");
        }
        r.get<uint32_t>("cloud_header_seq");r.get<uint32_t>("odom_header_seq");
        state.submap_records.emplace(id,r);state.record_offset=file.tellg();
    }
}
void ingest() {
    loadStreamingRecords();
    std::lock_guard<std::mutex> lock(mtx_sub_);
    while(!odom_buf_.empty()) {
        auto m=odom_buf_.front();odom_buf_.pop_front();++state.received["submap_pose"];
        double id=m->twist.covariance[0];
        if(!std::isfinite(id)||id<0||id!=std::floor(id)) throw std::runtime_error("invalid submap ID");
        if(size_t(id)>=keyframes_.size()) state.odoms.emplace(int(id),m);
    }
    while(!cloud_buf_.empty()) {
        auto m=cloud_buf_.front();cloud_buf_.pop_front();++state.received["clouds_submap"];
        if(keyframes_.empty()||m->header.stamp>keyframes_.back().KeyTime) state.clouds.emplace(m->header.stamp.toNSec(),m);
    }
    while(!interT_buf_.empty()) {
        auto m=interT_buf_.front();interT_buf_.pop_front();++state.received["inter_triangles"];
        auto c=decode(m);auto p=pointPair(c);
        if(p.second>=int(keyframes_.size())) state.adjacent.emplace(p.second,c);
    }
    // Upstream second-neighbor factors were disabled; explicitly account for them.
    state.received["inter2_triangles"]+=inter2T_buf_.size();inter2T_buf_.clear();
    if(state.verified_mode) {
        state.received["legacy_loops_ignored"]+=LCT_buf_.size()+loopclosure_buf_.size();
        LCT_buf_.clear();loopclosure_buf_.clear();
    }
    while(!state.verified_queue.empty()) {
        auto payload=state.verified_queue.front();state.verified_queue.pop_front();++state.received["verified_loops"];
        Pair p{-1,-1};
        try {
            factor_archive::Tree r;std::istringstream stream(payload);boost::property_tree::read_json(stream,r);
            p={r.get<int>("historical_id"),r.get<int>("current_id")};
            if(state.eof)throw std::runtime_error("verified_input_after_eof");
            if(r.get<int>("schema")!=1||r.get<std::string>("session_id")!=state.session_id||r.get<int>("revision")!=0||p.first<0||p.first>=p.second)
                throw std::runtime_error("invalid_verified_identity_or_revision");
            if(r.get<std::string>("loop_id")!=state.session_id+":loop:"+std::to_string(p.first)+":"+std::to_string(p.second)||
               r.get<std::string>("state")!="verified"||r.get<std::string>("measurement_convention")!="T_Bh_Bc"||
               r.get<std::string>("validator_version")!="d2-v1")throw std::runtime_error("invalid_verified_contract");
            factor_archive::readPose(r.get_child("measurement"));
            auto sig=factor_archive::numbers(r.get_child("sigmas"));
            if(sig.size()!=6)throw std::runtime_error("noise_dimension");
            for(int i=0;i<6;++i)if(sig[i]<(i<3?.005:.03)||sig[i]>(i<3?.2:1.))throw std::runtime_error("noise_out_of_bounds");
            if(r.get<std::string>("sigma_order")!="rx_ry_rz_tx_ty_tz"||r.get_child("sources").empty()||r.get_child("quality").empty())throw std::runtime_error("missing_verification_metadata");
            for(auto v:{std::make_pair(p.first,"historical_cloud_sha256"),std::make_pair(p.second,"current_cloud_sha256")})
                if(!state.submap_records.count(v.first)||r.get<std::string>(v.second)!=state.submap_records.at(v.first).get<std::string>("cloud_sha256"))throw std::runtime_error("submap_hash_mismatch");
            auto previous=state.verified_payload.find(p);
            if(previous!=state.verified_payload.end()) {event(p,previous->second==payload?"duplicate":"conflict",previous->second==payload?"verified_retransmission":"measurement_replacement_requires_E1");continue;}
            state.verified_payload.emplace(p,payload);state.verified.emplace(p,r);
            state.verified_audit<<payload<<std::endl;
            if(!state.verified_audit)throw std::runtime_error("verified_audit_write_failed");
            event(p,"verified","queued_pose3");
        }catch(const std::exception& e){if(std::string(e.what())=="verified_audit_write_failed")throw;++state.received["invalid_verified_loops"];event(p,"invalid",e.what());}
    }
    while(!LCT_buf_.empty()) {
        auto m=LCT_buf_.front();LCT_buf_.pop_front();++state.received["lc_triangles"];
        auto c=decode(m);auto p=pointPair(c);
        bool waiting=false;for(const auto& item:state.waiting)waiting|=item.pair==p;
        if(state.terminal.count(p)||waiting) {event(p,"duplicate","point_message_after_terminal_or_waiting");continue;}
        if(c->size()%6!=0||p.first<0||p.first>=p.second) {reject(p,"invalid_point_pair_layout_or_ids");continue;}
        for(const auto& v:c->points) if(!std::isfinite(v.x)||!std::isfinite(v.y)||!std::isfinite(v.z)) throw std::runtime_error("nonfinite correspondence");
        if(!state.points.emplace(p,c).second) event(p,"duplicate","point_message");
    }
    while(!loopclosure_buf_.empty()) {
        auto m=loopclosure_buf_.front();loopclosure_buf_.pop_front();++state.received["loop_transform"];
        double a=m->covariance[1],b=m->covariance[0];
        if(!std::isfinite(a)||!std::isfinite(b)||a!=std::floor(a)||b!=std::floor(b)) throw std::runtime_error("invalid loop IDs");
        Pair p{int(a),int(b)};
        if(state.terminal.count(p)) {event(p,"duplicate","proposal_after_terminal");continue;}
        if(p.first<0||p.first>=p.second) {reject(p,"invalid_loop_ids");continue;}
        bool waiting=false;for(const auto& c:state.waiting) waiting|=c.pair==p;
        if(waiting||!state.proposals.emplace(p,m).second) event(p,"duplicate","proposal");
        else event(p,"received","proposal");
    }
}
void baseUpdate() {
    if(gts_graph_.empty()&&gts_init_vals_.empty()) return;
    const auto started=std::chrono::steady_clock::now();
    ++state.transaction;
    std::string archived;
    if(state.rebuildable)archived=factor_archive::transaction(state.transaction,state.session_id,gts_graph_,gts_init_vals_,1);
    isam_->backup();
    gtsam::ISAM2Result result;
    try {result=isam_->update(gts_graph_,gts_init_vals_);isam_->update();}
    catch(...) {isam_->recover();clearStaging();throw;}
    archiveCommit(archived,result);clearStaging();refresh();
    recordTiming("base",started);
}
template<class Message> std::string messageSha(const Message& received,uint32_t original_seq) {
    // ROS publishers replace header.seq; restore only that transport field.
    auto message=received;message.header.seq=original_seq;
    std::vector<uint8_t> bytes(ros::serialization::serializationLength(message));
    ros::serialization::OStream stream(bytes.data(),bytes.size());ros::serialization::serialize(stream,message);
    unsigned char digest[SHA256_DIGEST_LENGTH];SHA256(bytes.data(),bytes.size(),digest);
    std::ostringstream out;out<<std::hex<<std::setfill('0');for(auto c:digest)out<<std::setw(2)<<int(c);return out.str();
}
bool addSubmap() {
    int id=keyframes_.size();auto o=state.odoms.find(id);auto adj=state.adjacent.find(id);
    if(o==state.odoms.end()||adj==state.adjacent.end()) return false;
    auto c=state.clouds.find(o->second->header.stamp.toNSec());if(c==state.clouds.end()) return false;
    if(state.verified_mode) {
        if(!state.submap_records.count(id)) {if(state.streaming_records)return false;throw std::runtime_error("submap_missing_from_verification_manifest");}
        const auto& r=state.submap_records.at(id);
        if(messageSha(*o->second,r.get<uint32_t>("odom_header_seq"))!=r.get<std::string>("odom_message_sha256")||messageSha(*c->second,r.get<uint32_t>("cloud_header_seq"))!=r.get<std::string>("cloud_message_sha256"))
            throw std::runtime_error("backend_input_does_not_match_immutable_submap");
    }
    KeyFrame k;k.KeyPose=OdomMsgToPose6D(o->second);k.KeyTime=o->second->header.stamp;k.KeyCloud=decode(c->second);
    if(k.KeyCloud->empty()) throw std::runtime_error("empty submap");
    if(!keyframes_.empty()) {
        const auto& last=keyframes_.back().KeyPose;
        passed_dis+=std::sqrt(std::pow(k.KeyPose.x-last.x,2)+std::pow(k.KeyPose.y-last.y,2)+std::pow(k.KeyPose.z-last.z,2));
    }
    keyframes_.push_back(k); addOdomFactor();
    if(id>0) {
        auto p=pointPair(adj->second);if(p!=Pair{id-1,id}) throw std::runtime_error("adjacent factor IDs do not match submap");
        const auto& a=adj->second->points[0];const auto& b=adj->second->points[1];
        bool absent=a.x==0&&a.y==0&&a.z==0&&b.x==0&&b.y==0&&b.z==0;
        if(!absent && state.adjacent_enabled) {
            if(adj->second->size()%2) throw std::runtime_error("odd adjacent correspondence message");
            insertKPPairConstraint(adj->second,id-1,id,1);
        }
    }
    state.odoms.erase(o);state.clouds.erase(c);state.adjacent.erase(adj);
    baseUpdate(); return true;
}
double maxPointResidual() {
    double largest=0;
    for(const auto& f:isam_->getFactorsUnsafe()) {
        auto p=boost::dynamic_pointer_cast<gtsam::TunningPointPairsFactor<gtsam::Pose3>>(f);
        if(p) {double e=p->unwhitenedError(gts_cur_vals_).norm(); if(!std::isfinite(e)) return std::numeric_limits<double>::infinity();largest=std::max(largest,e);}
    }
    return largest;
}
// Atomically replace old factors, after the loop transaction has committed.
// There is never a replacement graph left over for the next odometry update.
void compact(Pair anchor) {
    if(!state.compression_enabled) return;
    const auto started=std::chrono::steady_clock::now();
    gtsam::NonlinearFactorGraph replacement;
    gtsam::FactorIndices removed;
    std::set<Pair> old_loops;
    const auto& graph=isam_->getFactorsUnsafe();
    for(size_t i=0;i<graph.size();++i) {
        if(!graph[i]||graph[i]->keys().size()!=2) continue;
        int a=graph[i]->keys()[0],b=graph[i]->keys()[1];
        bool adjacent=a>=anchor.first&&b<=anchor.second&&b>a&&b-a<=2;
        bool old_loop=a!=anchor.first&&b!=anchor.second&&b-a>2;
        if(adjacent||old_loop) {removed.push_back(i);if(old_loop)old_loops.insert({a,b});}
    }
    std::vector<Pair> replacement_owner;
    for(int i=anchor.first;i<anchor.second;++i) {
        replacement.add(gtsam::BetweenFactor<gtsam::Pose3>(i,i+1,gts_cur_vals_.at<gtsam::Pose3>(i).between(gts_cur_vals_.at<gtsam::Pose3>(i+1)),pose_noise_));
        replacement_owner.push_back({-1,-1});
    }
    for(auto p:old_loops) {
        replacement.add(gtsam::BetweenFactor<gtsam::Pose3>(p.first,p.second,gts_cur_vals_.at<gtsam::Pose3>(p.first).between(gts_cur_vals_.at<gtsam::Pose3>(p.second)),pose_noise2_));
        replacement_owner.push_back(p);
    }
    if(removed.empty()) return;
    ++state.transaction; dump("before_replace");isam_->backup();
    gtsam::ISAM2Result result;
    try {result=isam_->update(replacement,gtsam::Values(),removed);isam_->update();}
    catch(const std::exception& e) {isam_->recover();refresh();event(anchor,"replacement_failed",e.what());dump("replacement_rollback");return;}
    for(auto p:old_loops) {
        std::vector<size_t> previous,next;
        for(auto i:removed) {auto it=state.factor_owner.find(i);if(it!=state.factor_owner.end()&&it->second==p)previous.push_back(i);}
        for(size_t j=0;j<replacement_owner.size();++j) if(replacement_owner[j]==p)next.push_back(result.newFactorsIndices.at(j));
        event(p,"replaced","accepted_estimate_compression",next,previous);
    }
    for(auto i:removed) state.factor_owner.erase(i);
    for(size_t j=0;j<replacement_owner.size();++j) if(replacement_owner[j].first>=0)state.factor_owner[result.newFactorsIndices.at(j)]=replacement_owner[j];
    ++state.replacements;refresh();dump("after_replace");
    recordTiming("compression",started);
}
void trial(const std::vector<Candidate>& batch) {
    if(batch.empty()) return;
    const auto started=std::chrono::steady_clock::now();
    ++state.transaction;dump("before_trial");
    const auto corners_before=cornerpairs_uomap;
    std::vector<std::pair<size_t,size_t>> ranges;
    clearStaging();
    for(const auto& c:batch) {
        size_t start=gts_graph_.size();insertKPPairConstraint(c.points,c.pair.first,c.pair.second,2);
        ranges.push_back({start,gts_graph_.size()});event(c.pair,"trial","geometrically_valid");
    }
    std::string archived;
    if(state.rebuildable) {
        std::vector<std::string> owners(gts_graph_.size());
        for(size_t j=0;j<batch.size();++j)for(size_t k=ranges[j].first;k<ranges[j].second;++k)
            owners[k]=state.session_id+":loop:"+std::to_string(batch[j].pair.first)+":"+std::to_string(batch[j].pair.second);
        archived=factor_archive::transaction(state.transaction,state.session_id,gts_graph_,gtsam::Values(),4,owners);
    }
    isam_->backup();gtsam::ISAM2Result result;double residual=0;std::string reason;
    try {
        result=isam_->update(gts_graph_,gtsam::Values());for(int i=0;i<4;++i)isam_->update();
        gts_cur_vals_=isam_->calculateEstimate();residual=maxPointResidual();dump("trial");
        if(!std::isfinite(residual)) reason="nonfinite_point_residual";
        else if(residual>residual_thr && state.loop_policy!="fixed_manifest")reason="point_residual_above_threshold";
#ifdef LTAOM_TRANSACTION_TEST_HOOKS
        for(const auto& c:batch)if(c.pair.second==state.force_reject)reason="test_forced_rejection";
#endif
    } catch(const std::exception& e) {reason=std::string("solver_exception: ")+e.what();}
    if(!reason.empty()) {
        isam_->recover();cornerpairs_uomap=corners_before;clearStaging();refresh();dump("rollback");
        for(const auto& c:batch)reject(c.pair,reason);
        times_opt_file << std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count() << " 0" << std::endl;
        if(state.loop_policy=="fixed_manifest") throw std::runtime_error("fixed manifest solve failed: "+reason);
        return;
    }
    archiveCommit(archived,result);clearStaging();refresh();
    for(size_t j=0;j<batch.size();++j) {
        const auto& c=batch[j];std::vector<size_t> indices;
        for(size_t k=ranges[j].first;k<ranges[j].second;++k) {size_t i=result.newFactorsIndices.at(k);indices.push_back(i);state.factor_owner[i]=c.pair;}
        state.terminal[c.pair]="accepted";++state.accepted;
        VOXEL_LOC loc;loc.x=c.pair.first;loc.y=c.pair.second;loc.z=0;lc_pose_uomap.emplace(loc,c.relative);
        event(c.pair,"accepted","committed",indices);
        auto p=GTSPoseToPose6D(c.relative);
        lc_file << c.pair.first << ' ' << c.pair.second << ' ' << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.roll << ' ' << p.pitch << ' ' << p.yaw << std::endl;
        pubLoopClosureVisualization(c.pair.first,c.pair.second);
    }
    ++loop_corr_counter;passed_dis_atlast_lc=passed_dis;
    times_opt_file << std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count() << " 1" << std::endl;
    dump("accepted");
    recordTiming("loop_without_compression",started);
    compact(batch.back().pair);
    pubPath();
}
bool processLoop() {
    // Order ready proposals by current submap, independent of map key ordering.
    auto chosen=state.proposals.end();
    for(auto it=state.proposals.begin();it!=state.proposals.end();++it) {
        if(it->first.second>=int(keyframes_.size())||!state.points.count(it->first))continue;
        if(chosen==state.proposals.end()||std::make_pair(it->first.second,it->first.first)<std::make_pair(chosen->first.second,chosen->first.first))chosen=it;
    }
    if(chosen==state.proposals.end()) return false;
    Pair p=chosen->first;auto msg=chosen->second;auto pts=state.points.at(p);
    state.proposals.erase(chosen);state.points.erase(p);
    if(state.terminal.count(p))return true;
    const auto& q=msg->pose.orientation;const auto& tr=msg->pose.position;
    double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if(!std::isfinite(norm)||std::abs(norm-1)>1e-3||!std::isfinite(tr.x)||!std::isfinite(tr.y)||!std::isfinite(tr.z)) {reject(p,"invalid_transform");return true;}
    auto world=GeoPoseMsgToGTSPose(msg->pose);
    auto prev=Pose6DToGTSPose(keyframes_[p.first].KeyPose),cur=Pose6DToGTSPose(keyframes_[p.second].KeyPose);
    auto relative=prev.inverse()*world*cur;
    if(!state.loops_enabled || state.loop_policy=="log_only") {
        state.terminal[p]="disabled";++state.skipped;event(p,"disabled","disabled_by_ablation");return true;
    }
    if(state.loop_policy=="fixed_manifest") {
        if(!state.fixed_pairs.count(p)) {
            state.terminal[p]="excluded";++state.skipped;event(p,"excluded","not_in_fixed_manifest");return true;
        }
        // Experimental fixed-measurement comparison: no state-dependent acceptance gates.
        trial({Candidate{p,relative,pts,0}});return true;
    }
    double overlap=calculateOverlapScore(keyframes_[p.second].KeyCloud,keyframes_[p.first].KeyCloud,GTSPoseToEigenM4f(cur),GTSPoseToEigenM4f(prev),GTSPoseToEigenM4f(relative),vs_for_ovlap,150000,p.first,p.second);
    if(!std::isfinite(overlap)||overlap<overlap_score_thr) {reject(p,"overlap_or_plane_check");return true;}
    double extent=std::max({x_range_max-x_range_min,y_range_max-y_range_min,z_range_max-z_range_min});
    bool large=state.accepted>2&&world.translation().norm()>.1*extent;
    if(large&&world.translation().norm()>.5*extent) {reject(p,"excessive_world_correction");return true;}
    Candidate c{p,relative,pts,overlap};
    if(!state.waiting.empty()) {auto batch=state.waiting;state.waiting.clear();batch.push_back(c);trial(batch);}
    else if(!state.accepted||large) {state.waiting.push_back(c);event(p,"pending_confirmation",!state.accepted?"first_loop":"large_correction");}
    else trial({c});
    return true;
}
#include "verified_pose3_backend.hpp"
void finishInput() {
    if(!state.eof)return;
    // Input producers must confirm all per-topic message counts before requesting
    // EOF. An unmatched item is recorded as a terminal failure, never silently lost.
    if(!state.odoms.empty()||!state.clouds.empty()||!state.adjacent.empty()) {state.fatal="incomplete_submap_at_eof";return;}
    for(const auto& p:state.proposals)reject(p.first,"missing_submap_or_correspondences_at_eof");
    state.proposals.clear();
    for(const auto& c:state.waiting) reject(c.pair,"unconfirmed_at_eof");
    state.waiting.clear();
    std::vector<Pair> orphan;for(const auto& p:state.points)orphan.push_back(p.first);
    for(auto p:orphan) reject(p,"missing_proposal_at_eof");
    state.points.clear();
    for(const auto& item:state.verified)reject(item.first,"missing_submap_at_eof");
    state.verified.clear();
    state.drained=true;
}
bool drain(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response) {
    std::lock_guard<std::mutex> lock(mtx_pgo_);
    if(multisession_mode!=0) {response.success=false;response.message="drain_supported_for_single_session_only";return true;}
    if(state.verified_mode && !state.eof) {
        int expected=-1;ros::param::get("/expected_verified_loops",expected);
        std::lock_guard<std::mutex> input_lock(mtx_sub_);
        int expected_submaps=-1;ros::param::get("/expected_submaps",expected_submaps);
        if(state.streaming_records&&(expected_submaps<0||state.submap_records.size()!=size_t(expected_submaps))) {
            response.success=false;response.message="await_expected_submaps";return true;
        }
        if(expected<0||state.received["verified_loops"]!=size_t(expected)||!state.verified_queue.empty()||keyframes_.size()!=state.submap_records.size()) {
            response.success=false;response.message="await_expected_verified_count_and_complete_submaps";return true;
        }
    }
    state.eof=true;
    response.success=state.drained&&state.fatal.empty();
    response.message=!state.fatal.empty()?state.fatal:(response.success?"drained":"drain_requested");
    status(); // Persist the same state acknowledged to the caller.
    return true;
}
void init(ros::NodeHandle& nh) {
    nh.param<bool>("backend_audit_factors",state.audit,false);
#ifdef LTAOM_ABLATION
    if(multisession_mode!=0) throw std::runtime_error("ablation supports single session only");
    nh.param<bool>("adjacent_factors_en",state.adjacent_enabled,true);
    nh.param<bool>("loop_factors_en",state.loops_enabled,true);
    nh.param<bool>("factor_compression_en",state.compression_enabled,true);
    nh.param<std::string>("loop_policy",state.loop_policy,"native");
    if(state.loop_policy!="native" && state.loop_policy!="fixed_manifest" && state.loop_policy!="log_only")
        throw std::runtime_error("unknown ablation loop policy");
    if(state.loop_policy=="fixed_manifest") {
        std::string path;nh.param<std::string>("fixed_loop_manifest",path,"");
        std::ifstream file(path);int a,b;
        if(!file) throw std::runtime_error("cannot read fixed loop manifest");
        while(file>>a>>b) {
            if(a<0||a>=b||!state.fixed_pairs.emplace(a,b).second) throw std::runtime_error("invalid fixed loop pair");
        }
        if(!file.eof()||state.fixed_pairs.empty()) throw std::runtime_error("invalid or empty fixed manifest");
    }
    state.timing.open(save_directory+"factor_timing.csv");
    state.timing << "transaction,stage,submaps,milliseconds\n";
#endif
    std::string mode;nh.param<std::string>("backend_mode",mode,"native");
    if(mode!="native"&&mode!="rebuildable")throw std::runtime_error("unknown backend_mode");
    state.rebuildable=(mode=="rebuildable");
    if(state.rebuildable) {
        bool feedback;nh.param<bool>("frontend_feedback_en",feedback,false);
        if(multisession_mode!=0||feedback)throw std::runtime_error("rebuildable mode requires single session and frontend feedback disabled");
        state.compression_enabled=false;
        nh.param<bool>("adjacent_factors_en",state.adjacent_enabled,false);
        nh.param<bool>("loop_factors_en",state.loops_enabled,false);
        nh.param<std::string>("session_id",state.session_id,"");
        if(state.session_id.empty()||state.session_id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
            throw std::runtime_error("session_id must be nonempty [A-Za-z0-9_-]");
        auto path=save_directory+"factor_transactions.jsonl";
        if(std::ifstream(path))throw std::runtime_error("refusing to overwrite factor archive");
        state.archive.open(path);if(!state.archive)throw std::runtime_error("cannot open factor archive");
    }
    std::string input_mode;nh.param<std::string>("loop_input_mode",input_mode,"legacy");
    if(input_mode!="legacy"&&input_mode!="verified_pose3")throw std::runtime_error("unknown_loop_input_mode");
    state.verified_mode=input_mode=="verified_pose3";
    if(state.verified_mode) {
        if(!state.rebuildable||!state.loops_enabled||state.loop_policy!="native")throw std::runtime_error("verified_pose3_requires_rebuildable_enabled_native_policy");
        nh.param<bool>("streaming_submap_records",state.streaming_records,false);
        std::string path;nh.param<std::string>("submap_records",path,"");state.record_path=path;
        if(path.empty())throw std::runtime_error("empty_submap_records_path");
        std::ifstream file(path);std::string line;
        if(!state.streaming_records && !file)throw std::runtime_error("missing_submap_records");
        while(!state.streaming_records && std::getline(file,line)) {
            factor_archive::Tree r;std::istringstream stream(line);boost::property_tree::read_json(stream,r);
            int id=r.get<int>("submap_id");
            if(id!=int(state.submap_records.size())||r.get<std::string>("session_id")!=state.session_id)throw std::runtime_error("invalid_submap_manifest");
            state.submap_records.emplace(id,r);
        }
        nh.param<double>("max_loop_normalized_residual",state.max_loop_normalized,8.);
        nh.param<double>("max_graph_normalized_residual",state.max_graph_normalized,20.);
        if(!(state.max_loop_normalized>0&&state.max_graph_normalized>0))throw std::runtime_error("invalid_residual_threshold");
        state.verified_audit.open(save_directory+"verified_inputs.jsonl");
        state.verified_subscriber=nh.subscribe<std_msgs::String>("/loop_pipeline/verified_loops",100,boost::function<void(const std_msgs::String::ConstPtr&)>([](const std_msgs::String::ConstPtr& m){
            std::lock_guard<std::mutex> lock(mtx_sub_);
            if(state.verified_queue.size()>=512) {ROS_FATAL("verified queue overflow");ros::shutdown();return;}
            state.verified_queue.push_back(m->data);
        }));
    }
#ifdef LTAOM_TRANSACTION_TEST_HOOKS
    nh.param<int>("test_force_reject_submap",state.force_reject,-1);
#endif
    state.ledger.open(save_directory+"loop_ledger.jsonl");
    if(!state.ledger)throw std::runtime_error("cannot open loop ledger");
    if(state.audit) {state.inventory.open(save_directory+"factor_inventory.csv");state.inventory << "transaction,stage,keyframes,index,type,key0,key1\n";}
    status();
}
void worker() {
    auto last=std::chrono::steady_clock::now();
    while(ros::ok()) {
        {
            std::lock_guard<std::mutex> lock(mtx_pgo_);
            if(state.fatal.empty())try {
                ingest();
                bool work=false;
                while(addSubmap()) {work=true;while(processLoop()){} while(processVerified()){} }
                while(processLoop())work=true;
                while(processVerified())work=true;
                if(!state.drained)finishInput();
                if(work||std::chrono::steady_clock::now()-last>std::chrono::milliseconds(200)) {status();last=std::chrono::steady_clock::now();}
            } catch(const std::exception& e) {state.fatal=e.what();ROS_ERROR_STREAM("Single-session backend stopped: " << state.fatal);status();}
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::lock_guard<std::mutex> lock(mtx_pgo_);dump("final");status();
}
} // namespace single_backend
