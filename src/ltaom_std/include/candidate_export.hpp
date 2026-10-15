#include <ltaom_transport/ros2_transport.hpp>
#pragma once
#include <sstream>
#include <iomanip>
// Explicit versioned transport; IDs never occupy covariance/intensity fields.
// This is retrieved evidence only, before fine matching and geometric gates.
inline std::string stdCandidateBatch(const std::string& session,int current,
                                    const std::vector<STDMatchList>& candidates,
                                    const ConfigSetting* config=nullptr) {
    std::ostringstream s;s<<std::setprecision(17);
    s<<"{\"schema\":1,\"session_id\":\""<<session<<"\",\"source\":\"STD\",\"current_id\":"<<current<<",\"stage\":\"retrieved\",\"candidates\":[";
    bool first=true;
    for(size_t i=0;i<candidates.size();++i){auto& c=candidates[i];
        // Native search fills unused top-K slots with empty lists and an
        // uninitialized frame ID. Never read or export those placeholders.
        if(c.match_list_.empty())continue;
        if(c.match_frame_<0||c.match_frame_>=current)throw std::runtime_error("invalid STD candidate ID");
        s<<(first?"":",")<<"{\"historical_id\":"<<c.match_frame_<<",\"rank\":"<<i
         <<",\"score_name\":\"triangle_match_count\",\"score\":"<<c.match_list_.size()
         <<",\"higher_is_better\":true";
        bool valid=false;Eigen::Matrix3d rotation=Eigen::Matrix3d::Identity();Eigen::Vector3d translation=Eigen::Vector3d::Zero();
        std::vector<std::pair<STD,STD>> good,bad;
        if(config && c.match_list_.size()>=4) {
            auto matches=c.match_list_;std::fstream debug;
            fine_loop_detection_tbb(*config,matches,valid,rotation,translation,good,bad,debug);
            valid=valid && rotation.allFinite() && translation.allFinite() &&
                std::abs(rotation.determinant()-1)<1e-6 &&
                (rotation.transpose()*rotation-Eigen::Matrix3d::Identity()).norm()<1e-6;
        }
        s<<",\"has_initial_transform\":"<<(valid?"true":"false")<<",\"initial_inliers\":"<<good.size();
        if(valid) {
            s<<",\"initial_frame\":\"world_current_to_history\",\"initial_transform\":[";
            for(int row=0;row<4;++row)for(int col=0;col<4;++col)
                s<<(row||col?",":"")<<(row==3?(col==3?1.:0.):(col==3?translation[row]:rotation(row,col)));
            s<<']';
        }
        s<<'}';first=false;}
    s<<"]}";return s.str();
}
