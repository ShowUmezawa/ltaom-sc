// Fixed local-cloud registration. Quatro coarse initialization follows SC-QN's
// coarse-to-fine composition; independent quality gates are implemented here.
#include <nano_gicp/nano_gicp.hpp>
#include <quatro/quatro_module.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/property_tree/json_parser.hpp>
#include <Eigen/Eigenvalues>
#include <chrono>
#include <fstream>
#include <iomanip>
using Tree=boost::property_tree::ptree;
using Point=pcl::PointXYZI;
using Cloud=pcl::PointCloud<Point>;
using Mat=Eigen::Matrix4d;
Mat matrix(const Tree& tree) {
    Mat m;int i=0;for(const auto& v:tree){if(i>=16)throw std::runtime_error("matrix_length");m(i/4,i%4)=v.second.get_value<double>();++i;}
    if(i!=16||!m.allFinite()||(m.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()>1e-8 ||
       (m.block<3,3>(0,0).transpose()*m.block<3,3>(0,0)-Eigen::Matrix3d::Identity()).norm()>1e-5||std::abs(m.block<3,3>(0,0).determinant()-1)>1e-5)
        throw std::runtime_error("invalid_SE3");
    return m;
}
Tree values(const Mat& m) {Tree a;for(int i=0;i<16;++i){Tree v;v.put("",m(i/4,i%4));a.push_back({"",v});}return a;}
Cloud::Ptr load(const std::string& path,double voxel) {
    Cloud::Ptr raw(new Cloud),out(new Cloud);
    if(pcl::io::loadPCDFile(path,*raw)<0)throw std::runtime_error("pcd_read_failed");
    for(const auto& p:*raw)if(!pcl::isFinite(p))throw std::runtime_error("nonfinite_cloud");
    pcl::VoxelGrid<Point> filter;filter.setInputCloud(raw);filter.setLeafSize(voxel,voxel,voxel);filter.filter(*out);return out;
}
double angle(const Mat& a){return Eigen::AngleAxisd(a.block<3,3>(0,0)).angle();}
Tree check(const Cloud::Ptr& source,const Cloud::Ptr& target,const Mat& transform,const Mat& initial,const Tree& cfg,bool converged,const std::string& correspondence_path) {
    Tree r;r.put("accepted",false);r.put("reason","not_converged");
    if(!converged)return r;
    matrix(values(transform));
    Cloud::Ptr aligned(new Cloud);pcl::transformPointCloud(*source,*aligned,transform);
    pcl::KdTreeFLANN<Point> dst,src;dst.setInputCloud(target);src.setInputCloud(aligned);
    pcl::NormalEstimation<Point,pcl::Normal> ne;ne.setInputCloud(target);ne.setKSearch(20);
    pcl::PointCloud<pcl::Normal> normals;ne.compute(normals);
    const double threshold=cfg.get<double>("inlier_distance_m");
    std::vector<int> idx(1);std::vector<float> dist(1);size_t forward=0,reverse=0,normal_count=0;
    double squared=0,plane_squared=0;
    Eigen::Vector3d mean=Eigen::Vector3d::Zero();Eigen::Matrix3d moment=Eigen::Matrix3d::Zero();
    std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> normal_pairs;
    for(const auto& p:*aligned) {
        if(dst.nearestKSearch(p,1,idx,dist)!=1||dist[0]>threshold*threshold)continue;
        ++forward;squared+=dist[0];Eigen::Vector3d v=p.getVector3fMap().cast<double>();mean+=v;moment+=v*v.transpose();
        const auto& n=normals[idx[0]];Eigen::Vector3d normal(n.normal_x,n.normal_y,n.normal_z);
        if(!normal.allFinite()||normal.norm()<.9)continue;
        normal.normalize();auto q=target->at(idx[0]).getVector3fMap().cast<double>().eval();
        plane_squared+=std::pow(normal.dot(v-q),2);++normal_count;normal_pairs.emplace_back(v,normal);
    }
    for(const auto& p:*target)if(src.nearestKSearch(p,1,idx,dist)==1&&dist[0]<=threshold*threshold)++reverse;
    const double f=double(forward)/source->size(),b=double(reverse)/target->size();
    r.put("forward_overlap",f);r.put("reverse_overlap",b);r.put("correspondences",forward);
    if(forward<static_cast<size_t>(cfg.get<int>("min_points"))||normal_count<100){r.put("reason","insufficient_correspondences");return r;}
    mean/=forward;Eigen::Matrix3d covariance=moment/forward-mean*mean.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> spread(covariance);
    double extent=2*std::sqrt(std::max(0.,spread.eigenvalues()[1]));
    double radius=std::sqrt(std::max(1.,covariance.trace()));
    Eigen::Matrix<double,6,6> H=Eigen::Matrix<double,6,6>::Zero();
    for(auto& pair:normal_pairs){Eigen::Matrix<double,6,1> j;j.head<3>()=((pair.first-mean)/radius).cross(pair.second);j.tail<3>()=pair.second;H+=j*j.transpose();}
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eig(H/normal_count);
    double ratio=std::max(0.,eig.eigenvalues()[0])/std::max(1e-12,eig.eigenvalues()[5]);
    double rmse=std::sqrt(squared/forward),plane=std::sqrt(plane_squared/normal_count);
    Mat change=transform*initial.inverse();double translation=change.block<3,1>(0,3).norm(),rotation=angle(change);
    r.put("inlier_rmse_m",rmse);r.put("point_plane_rmse_m",plane);r.put("spatial_extent_m",extent);
    r.put("normalized_hessian_ratio",ratio);r.put("initial_translation_change_m",translation);r.put("initial_rotation_change_rad",rotation);
    std::string reason="passed";
    if(std::min(f,b)<cfg.get<double>("min_overlap"))reason="low_bidirectional_overlap";
    else if(rmse>cfg.get<double>("max_inlier_rmse_m"))reason="correspondence_distance";
    else if(plane>cfg.get<double>("max_point_plane_rmse_m"))reason="point_plane_residual";
    else if(extent<cfg.get<double>("min_spatial_extent_m"))reason="concentrated_correspondences";
    else if(ratio<cfg.get<double>("min_normalized_hessian_ratio"))reason="geometric_degeneracy";
    else if(translation>cfg.get<double>("max_initial_translation_change_m")||rotation>cfg.get<double>("max_initial_rotation_change_rad"))reason="excessive_initial_change";
    if(reason=="passed"&&!correspondence_path.empty()) {
        std::ofstream file(correspondence_path);file<<std::setprecision(9)<<"source_x,source_y,source_z,target_x,target_y,target_z\n";
        for(size_t i=0;i<aligned->size();++i)if(dst.nearestKSearch(aligned->at(i),1,idx,dist)==1&&dist[0]<=threshold*threshold) {
            const auto& a=source->at(i);const auto& b=target->at(idx[0]);
            file<<a.x<<','<<a.y<<','<<a.z<<','<<b.x<<','<<b.y<<','<<b.z<<'\n';
        }
        if(!file)throw std::runtime_error("correspondence_write_failed");
    }
    r.put("reason",reason);r.put("accepted",reason=="passed");return r;
}
int main(int argc,char** argv) {
    if(argc!=2){std::cerr<<"usage: registration config.json\n";return 1;}
    Tree cfg;boost::property_tree::read_json(argv[1],cfg);
    auto* output=std::cout.rdbuf();std::cout.rdbuf(std::cerr.rdbuf());std::ostream result(output);
    std::string line;
    while(std::getline(std::cin,line)) {
        auto start=std::chrono::steady_clock::now();Tree r;
        try {
            Tree q;std::istringstream in(line);boost::property_tree::read_json(in,q);
            auto source=load(q.get<std::string>("source"),cfg.get<double>("voxel_m"));
            auto target=load(q.get<std::string>("target"),cfg.get<double>("voxel_m"));
            r.put("source_points",source->size());r.put("target_points",target->size());
            if(std::min(source->size(),target->size())<static_cast<size_t>(cfg.get<int>("min_points")))throw std::runtime_error("insufficient_points");
            Mat initial=matrix(q.get_child("initial_transform"));
            if(q.get<bool>("quatro",false)) {
                if(!q.get<bool>("gravity_valid",false))throw std::runtime_error("unvalidated_gravity");
                Mat Gs=matrix(q.get_child("source_gravity")),Gt=matrix(q.get_child("target_gravity"));
                double yaw=q.get<double>("yaw_rad");Mat Y=Mat::Identity();Y.block<3,3>(0,0)=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
                Cloud sg,tg;pcl::transformPointCloud(*source,sg,(Y*Gs).eval());pcl::transformPointCloud(*target,tg,Gt);
                quatro<Point> coarse(1.2,2.0,.4,1.4,.0001,50,false,true,80.,500);
                bool valid=false;Mat Q=coarse.align(sg,tg,valid);
                if(!valid)throw std::runtime_error("quatro_failed");
                initial=Gt.inverse()*matrix(values(Q))*Y*Gs;r.put("coarse_method","Quatro");
            }else r.put("coarse_method","provided_SE3");
            nano_gicp::NanoGICP<Point,Point> gicp;
            gicp.setNumThreads(cfg.get<int>("threads"));gicp.setCorrespondenceRandomness(20);
            gicp.setMaximumIterations(64);gicp.setMaxCorrespondenceDistance(cfg.get<double>("max_correspondence_m"));
            gicp.setTransformationEpsilon(1e-4);gicp.setEuclideanFitnessEpsilon(1e-5);
            // Explicit composition: fine correction maps prealigned source to target.
            Cloud::Ptr prealigned(new Cloud);pcl::transformPointCloud(*source,*prealigned,initial);
            gicp.setInputSource(prealigned);gicp.calculateSourceCovariances();gicp.setInputTarget(target);gicp.calculateTargetCovariances();
            Cloud aligned;gicp.align(aligned);Mat transform=gicp.getFinalTransformation().cast<double>()*initial;
            auto quality=check(source,target,transform,initial,cfg,gicp.hasConverged(),q.get<std::string>("correspondences_path",""));
            r.put("accepted",quality.get<bool>("accepted"));r.put("reason",quality.get<std::string>("reason"));
            r.add_child("quality",quality);r.add_child("transform",values(transform));r.add_child("refinement_initial",values(initial));
        }catch(const std::exception& e){r.put("accepted",false);r.put("reason",e.what());}
        r.put("milliseconds",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
        std::ostringstream out;boost::property_tree::write_json(out,r,false);result<<out.str()<<std::flush;
    }
    std::cout.rdbuf(output);
}
