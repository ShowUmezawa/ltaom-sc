#pragma once
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include "TunningPointPairsFactor.h"
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>

// Version 1 archives only original measurements. No optimized pose is used as
// a factor measurement. One complete JSON line is one committed transaction.
namespace factor_archive {
using Tree = boost::property_tree::ptree;
inline void pose(std::ostream& out, const gtsam::Pose3& p) {
    const auto q=p.rotation().toQuaternion(); const auto t=p.translation();
    out << '[' << t.x() << ',' << t.y() << ',' << t.z() << ','
        << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ']';
}
inline std::vector<double> numbers(const Tree& p) {
    std::vector<double> v;for(const auto& x:p) {double n=x.second.get_value<double>();
        if(!std::isfinite(n)) throw std::runtime_error("nonfinite archive measurement");v.push_back(n);}return v;
}
inline gtsam::Pose3 readPose(const Tree& p) {
    auto v=numbers(p);if(v.size()!=7)throw std::runtime_error("invalid pose length");
    double n=0;for(int i=3;i<7;++i)n+=v[i]*v[i];
    if(std::abs(n-1)>1e-8)throw std::runtime_error("invalid archive quaternion");
    return {gtsam::Rot3::Quaternion(v[6],v[3],v[4],v[5]),gtsam::Point3(v[0],v[1],v[2])};
}
inline std::string factor(const gtsam::NonlinearFactor::shared_ptr& f,
                          const std::string& id, const std::string& owner) {
    std::ostringstream s;s<<std::setprecision(17);
    auto nf=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(f);
    if(!nf)throw std::runtime_error("unsupported archived factor");
    auto noise=nf->noiseModel();std::string robust="none";
    if(auto r=boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(noise)) {
        if(!r->robust()->equals(*gtsam::noiseModel::mEstimator::Cauchy::Create(1)))
            throw std::runtime_error("unsupported robust noise: archive supports Cauchy(1) only");
        robust="cauchy_1_block";noise=r->noise();
    }
    auto d=boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(noise);
    if(!d)throw std::runtime_error("archive requires diagonal noise");
    s<<"{\"factor_id\":\""<<id<<"\",\"loop_id\":\""<<owner<<"\",\"revision\":0,\"keys\":[";
    for(size_t i=0;i<f->keys().size();++i)s<<(i?",":"")<<f->keys()[i];
    s<<"],\"robust\":\""<<robust<<"\",\"sigmas\":[";
    auto sig=d->sigmas();for(int i=0;i<sig.size();++i){if(!(sig[i]>0))throw std::runtime_error("invalid sigma");s<<(i?",":"")<<sig[i];}
    s<<"],\"kind\":\"";
    if(auto a=boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(f)) {s<<"prior\",\"measurement\":";pose(s,a->prior());}
    else if(auto a=boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(f)) {s<<"between\",\"measurement\":";pose(s,a->measured());}
    else if(auto a=boost::dynamic_pointer_cast<gtsam::TunningPointPairsFactor<gtsam::Pose3>>(f)) {
        s<<"point_pair\",\"measurement\":[";for(int i=0;i<3;++i)s<<(i?",":"")<<a->sourcePoint()[i];
        for(int i=0;i<3;++i)s<<','<<a->measured()[i];s<<']';
    } else throw std::runtime_error("unsupported factor kind");
    s<<'}';return s.str();
}
inline gtsam::NonlinearFactor::shared_ptr readFactor(const Tree& p) {
    std::vector<gtsam::Key> keys;for(const auto& k:p.get_child("keys"))keys.push_back(k.second.get_value<gtsam::Key>());
    auto sig=numbers(p.get_child("sigmas"));gtsam::Vector v(sig.size());
    for(size_t i=0;i<sig.size();++i){if(!(sig[i]>0))throw std::runtime_error("invalid sigma");v[i]=sig[i];}
    gtsam::SharedNoiseModel noise=gtsam::noiseModel::Diagonal::Sigmas(v);
    auto robust=p.get<std::string>("robust");
    if(robust=="cauchy_1_block")noise=gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(1),noise);
    else if(robust!="none")throw std::runtime_error("unknown robust model");
    auto kind=p.get<std::string>("kind");
    if(kind=="prior"&&keys.size()==1&&sig.size()==6)return boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(keys[0],readPose(p.get_child("measurement")),noise);
    if(kind=="between"&&keys.size()==2&&sig.size()==6)return boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(keys[0],keys[1],readPose(p.get_child("measurement")),noise);
    if(kind=="point_pair"&&keys.size()==2&&sig.size()==3) {
        auto m=numbers(p.get_child("measurement"));if(m.size()!=6)throw std::runtime_error("point measurement length");
        return boost::make_shared<gtsam::TunningPointPairsFactor<gtsam::Pose3>>(keys[0],keys[1],gtsam::Point3(m[0],m[1],m[2]),gtsam::Point3(m[3],m[4],m[5]),noise);
    }
    throw std::runtime_error("unknown factor kind or dimensions");
}
inline std::string transaction(size_t tx, const std::string& session,
    const gtsam::NonlinearFactorGraph& graph,const gtsam::Values& initial,
    int empty_updates,const std::vector<std::string>& owners={}) {
    std::ostringstream s;s<<std::setprecision(17);
    s<<"{\"schema\":1,\"session_id\":\""<<session<<"\",\"graph_epoch\":0,\"transaction\":"<<tx
     <<",\"empty_updates\":"<<empty_updates<<",\"initial\":[";
    bool first=true;for(auto k:initial.keys()){s<<(first?"":",")<<"{\"key\":"<<k<<",\"pose\":";pose(s,initial.at<gtsam::Pose3>(k));s<<'}';first=false;}
    s<<"],\"factors\":[";
    for(size_t i=0;i<graph.size();++i)s<<(i?",":"")<<factor(graph[i],session+":factor:"+std::to_string(tx)+":"+std::to_string(i),owners.empty()?"":owners.at(i));
    s<<"]}";return s.str();
}
} // namespace factor_archive
