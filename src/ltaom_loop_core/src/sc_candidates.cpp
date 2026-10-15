#include "Scancontext.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
using Tree=boost::property_tree::ptree;
// Streaming interface: stdin contains committed SubmapRecord JSON lines;
// stdout contains exactly one explicit CandidateBatch per input, including empty.
int main(int argc,char** argv) {
 try {
    if(argc!=6)throw std::runtime_error("usage: sc_candidates archive_dir exclude_recent min_seconds top_k max_score");
    std::string root=argv[1];int gap=std::stoi(argv[2]),top=std::stoi(argv[4]);double dt=std::stod(argv[3]),threshold=std::stod(argv[5]);
    if(gap<1||top<1||dt<0||!std::isfinite(threshold)||threshold<0||threshold>2)throw std::runtime_error("invalid SC configuration");
    auto* stdout_buffer=std::cout.rdbuf();std::cout.rdbuf(std::cerr.rdbuf());
    std::ostream result_stream(stdout_buffer);
    SCManager sc;std::vector<Eigen::MatrixXd> desc;KeyMat keys;std::vector<double> times;std::string line,session;
    result_stream<<std::setprecision(17);
    while(std::getline(std::cin,line)) {
        Tree r;std::istringstream input(line);boost::property_tree::read_json(input,r);
        int id=r.get<int>("submap_id");double time=r.get<double>("state_time");
        if(session.empty())session=r.get<std::string>("session_id");
        if(id!=int(desc.size())||session!=r.get<std::string>("session_id")||!std::isfinite(time)||(!times.empty()&&time<=times.back()))throw std::runtime_error("out-of-order or mixed-session submap");
        pcl::PointCloud<SCPointType> cloud;
        if(pcl::io::loadPCDFile(root+"/"+r.get<std::string>("cloud_path"),cloud)<0||cloud.empty())throw std::runtime_error("invalid local PCD");
        Eigen::Matrix3d gravity;int j=0;for(auto& a:r.get_child("gravity_alignment_R")){if(j>=9)throw std::runtime_error("gravity matrix length");gravity(j/3,j%3)=a.second.get_value<double>();++j;}
        if(j!=9||!gravity.allFinite()||(gravity.transpose()*gravity-Eigen::Matrix3d::Identity()).norm()>1e-7||std::abs(gravity.determinant()-1)>1e-7)throw std::runtime_error("invalid gravity alignment");
        pcl::PointCloud<SCPointType> valid;
        for(auto p:cloud){Eigen::Vector3d xyz=gravity*Eigen::Vector3d(p.x,p.y,p.z);
            if(!xyz.allFinite())throw std::runtime_error("nonfinite local cloud");
            // Avoid upstream atan(0/0); points outside the polar support carry no descriptor information.
            double radial=xyz.head<2>().norm();if(radial<1e-6||radial>sc.PC_MAX_RADIUS)continue;
            p.x=xyz.x();p.y=xyz.y();p.z=xyz.z();valid.push_back(p);}
        Eigen::MatrixXd current=valid.empty()?Eigen::MatrixXd::Zero(sc.PC_NUM_RING,sc.PC_NUM_SECTOR):sc.makeScancontext(valid);
        auto key=eig2stdvec(sc.makeRingkeyFromScancontext(current));
        KeyMat eligible;std::vector<int> mapping;
        for(int h=0;h<id;++h)if(id-h>gap&&time-times[h]>=dt){eligible.push_back(keys[h]);mapping.push_back(h);}
        struct Hit{int id;double score,yaw;};std::vector<Hit> hits;
        if(!eligible.empty()&&current.norm()>0) {
            InvKeyTree tree(sc.PC_NUM_RING,eligible,10);int k=std::min<int>(top,eligible.size());
            std::vector<size_t> indices(k);std::vector<float> distances(k);
            nanoflann::KNNResultSet<float> result(k);result.init(indices.data(),distances.data());
            tree.index->findNeighbors(result,key.data(),nanoflann::SearchParams(10));
            for(int n=0;n<k;++n){int h=mapping.at(indices[n]);auto score=sc.distanceBtnScanContext(current,desc[h]);
                if(!std::isfinite(score.first)||score.first>threshold)continue;
                // Upstream shifts HISTORY to align CURRENT. Invert for T_history_current.
                double yaw=-score.second*2*M_PI/sc.PC_NUM_SECTOR;yaw=std::atan2(std::sin(yaw),std::cos(yaw));
                hits.push_back({h,score.first,yaw});}
            std::sort(hits.begin(),hits.end(),[](const Hit&a,const Hit&b){return a.score!=b.score?a.score<b.score:a.id<b.id;});
        }
        result_stream<<"{\"schema\":1,\"session_id\":\""<<session<<"\",\"source\":\"SC\",\"current_id\":"<<id<<",\"stage\":\"retrieved\",\"candidates\":[";
        for(size_t n=0;n<hits.size();++n){auto h=hits[n];result_stream<<(n?",":"")<<"{\"historical_id\":"<<h.id<<",\"rank\":"<<n<<",\"score_name\":\"scan_context_cosine_distance\",\"score\":"<<h.score<<",\"higher_is_better\":false,\"yaw_current_to_history_rad\":"<<h.yaw<<",\"yaw_frame\":\"gravity_aligned\",\"has_initial_transform\":false}";}
        result_stream<<"]}"<<std::endl;
        // Query-before-insert makes self/future matches impossible.
        desc.push_back(current);keys.push_back(key);times.push_back(time);
    }
    std::cout.rdbuf(stdout_buffer);
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
