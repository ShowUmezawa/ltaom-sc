#include "factor_archive.hpp"
#include <iostream>
int main(int argc,char** argv) {
 try {
    if(argc!=3)throw std::runtime_error("usage: rebuild_factor_archive transactions.jsonl output.json");
    std::ifstream input(argv[1]);if(!input)throw std::runtime_error("archive not found");
    if(std::ifstream(argv[2]))throw std::runtime_error("refusing to overwrite output");
    gtsam::ISAM2Params params;params.relinearizeThreshold=.1;params.relinearizeSkip=1;
    gtsam::ISAM2 solver(params);std::string line,session;size_t previous=0,count=0;std::set<std::string> ids;
    while(std::getline(input,line)) {
        if(line.empty())throw std::runtime_error("empty archive record");
        factor_archive::Tree t;std::istringstream in(line);boost::property_tree::read_json(in,t);
        if(t.get<int>("schema")!=1||t.get<int>("graph_epoch")!=0)throw std::runtime_error("unsupported schema/epoch");
        if(session.empty())session=t.get<std::string>("session_id");
        if(session!=t.get<std::string>("session_id"))throw std::runtime_error("mixed sessions");
        auto tx=t.get<size_t>("transaction");if(tx<=previous)throw std::runtime_error("unordered transaction");previous=tx;
        gtsam::Values initial;for(const auto& v:t.get_child("initial"))initial.insert(v.second.get<gtsam::Key>("key"),factor_archive::readPose(v.second.get_child("pose")));
        gtsam::NonlinearFactorGraph graph;for(const auto& f:t.get_child("factors")) {
            if(!ids.insert(f.second.get<std::string>("factor_id")).second)throw std::runtime_error("duplicate factor_id");
            graph.push_back(factor_archive::readFactor(f.second));
        }
        auto update=solver.update(graph,initial);
        std::vector<size_t> recorded;for(const auto& i:t.get_child("factor_indices"))recorded.push_back(i.second.get_value<size_t>());
        if(recorded!=update.newFactorsIndices)throw std::runtime_error("factor index reconstruction mismatch");
        int n=t.get<int>("empty_updates");if(n<0||n>10)throw std::runtime_error("invalid update count");
        for(int j=0;j<n;++j)solver.update();++count;
    }
    if(!count)throw std::runtime_error("empty archive");
    auto values=solver.calculateEstimate();std::ofstream out(argv[2]);out<<std::setprecision(17);
    out<<"{\"transactions\":"<<count<<",\"factors\":"<<ids.size()<<",\"poses\":[";
    bool first=true;for(auto k:values.keys()){out<<(first?"":",")<<"{\"key\":"<<k<<",\"pose\":";factor_archive::pose(out,values.at<gtsam::Pose3>(k));out<<'}';first=false;}
    out<<"]}\n";if(!out)throw std::runtime_error("failed to write rebuilt graph");
    std::cout<<"Rebuilt "<<count<<" transactions, "<<ids.size()<<" factors\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
