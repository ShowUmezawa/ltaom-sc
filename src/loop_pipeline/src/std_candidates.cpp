#include "include/std.h"
#include "include/candidate_export.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <pcl/io/pcd_io.h>
#include <sstream>
// Offline counterpart of the native pre-verification export hook. Re-extracts
// STD from saved world clouds; it does not replay or alter the LIO frontend.
int main(int argc,char** argv){
 try {
    if(argc!=3)throw std::runtime_error("usage: std_candidates archive_dir std_config.yaml");
    std::string root=argv[1],config=argv[2];ConfigSetting cfg;
    auto* stdout_buffer=std::cout.rdbuf();std::cout.rdbuf(std::cerr.rdbuf());
    load_config_setting(config,cfg);
    std::unordered_map<STD_LOC,std::vector<STD>> database;std::string line,session;int expected=0;
    while(std::getline(std::cin,line)){
        boost::property_tree::ptree r;std::istringstream in(line);boost::property_tree::read_json(in,r);
        int id=r.get<int>("submap_id");if(session.empty())session=r.get<std::string>("session_id");
        if(id!=expected++||id>=20000||session!=r.get<std::string>("session_id"))throw std::runtime_error("invalid STD submap sequence or native 20000-slot limit");
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if(pcl::io::loadPCDFile(root+"/"+r.get<std::string>("world_cloud_path"),*cloud)<0||cloud->empty())throw std::runtime_error("invalid world PCD");
        std::unordered_map<VOXEL_LOC,OctoTree*> voxels;init_voxel_map(cfg,*cloud,voxels);
        std::vector<Plane*> projected,merged;get_project_plane(cfg,voxels,projected);
        if(projected.empty()){
            Plane* p=new Plane;p->normal_<<0,0,1;p->center_<<cloud->front().x,cloud->front().y,cloud->front().z;merged.push_back(p);
        }else{
            std::sort(projected.begin(),projected.end(),plane_greater_sort);merge_plane(cfg,projected,merged);
            std::sort(merged.begin(),merged.end(),plane_greater_sort);
        }
        std::vector<BinaryDescriptor> binary;binary_extractor(cfg,merged,cloud,binary);
        std::vector<STD> descriptors;generate_std(cfg,binary,id,descriptors);
        std::vector<STDMatchList> candidates;candidate_searcher_old(cfg,database,descriptors,candidates);
        std::ostream result(stdout_buffer);result<<stdCandidateBatch(session,id,candidates,&cfg)<<std::endl;
        add_STD(database,descriptors);
        std::set<Plane*> planes(merged.begin(),merged.end());
        std::function<void(OctoTree*)> destroy=[&](OctoTree* t){if(!t)return;planes.insert(t->plane_ptr_);for(auto* child:t->leaves_)destroy(child);delete t;};
        for(auto& v:voxels)destroy(v.second);
        for(auto* p:planes)delete p;
    }
    std::cout.rdbuf(stdout_buffer);
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
