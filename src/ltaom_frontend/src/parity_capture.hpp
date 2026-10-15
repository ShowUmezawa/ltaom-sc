// Diagnostic-only capture. Called on the frontend owner thread, outside OpenMP.
// Files contain row-major IEEE754 doubles, with no PointType padding bytes.
#include <cstdint>
#include <cstdlib>
#include <cstring>
static bool parity_enabled(){static const bool on=[](){const char* v=std::getenv("LTAOM_PARITY_CAPTURE");return v && std::strcmp(v,"1")==0;}();return on;}
#include <fstream>
#include <iomanip>
#include <stdexcept>
static unsigned parity_iteration=0;
static bool parity_active(){return parity_enabled() && consumed_lidar_count<=40;}
static std::string parity_name(const std::string& name){
    return save_directory+"parity_"+std::to_string(consumed_lidar_count)+"_"+name+".bin";
}
static void parity_values(const std::string& name,const std::vector<double>& v){
    if(!parity_active())return;
    std::ofstream f(parity_name(name),std::ios::binary);
    f.write(reinterpret_cast<const char*>(v.data()),v.size()*sizeof(double));
    if(!f)throw std::runtime_error("parity capture write failed");
}
template<class M> static void parity_matrix(const std::string& name,const M& m){
    if(!parity_active())return;
    std::vector<double> v;v.reserve(m.size());
    for(int r=0;r<m.rows();++r)for(int c=0;c<m.cols();++c)v.push_back(m(r,c));
    parity_values(name,v);
}
template<class C> static void parity_cloud(const std::string& name,const C& cloud){
    if(!parity_active())return;
    std::vector<double> v;v.reserve(cloud.size()*5);
    for(const auto& p:cloud.points){v.push_back(p.x);v.push_back(p.y);v.push_back(p.z);v.push_back(p.intensity);v.push_back(p.curvature);}
    parity_values(name,v);
}
static void parity_state(const std::string& name,const state_ikfom& s){
    if(!parity_active())return;
    std::vector<double> v;
    for(int i=0;i<3;++i)v.push_back(s.pos[i]);
    for(int i=0;i<4;++i)v.push_back(s.rot.coeffs()[i]);
    for(int i=0;i<3;++i)v.push_back(s.vel[i]);
    for(int i=0;i<3;++i)v.push_back(s.bg[i]);
    for(int i=0;i<3;++i)v.push_back(s.ba[i]);
    for(int i=0;i<3;++i)v.push_back(s.grav[i]);
    for(int i=0;i<4;++i)v.push_back(s.offset_R_L_I.coeffs()[i]);
    for(int i=0;i<3;++i)v.push_back(s.offset_T_L_I[i]);
    parity_values(name,v);
}
static uint64_t parity_hash(uint64_t h,const void* data,size_t n){
    const auto* p=static_cast<const unsigned char*>(data);
    for(size_t i=0;i<n;++i){h^=p[i];h*=1099511628211ULL;}return h;
}
static void parity_input(const MeasureGroup& m){
    if(!parity_enabled())return;
    static std::ofstream f(save_directory+"parity_inputs.csv");
    if(consumed_lidar_count==1)f<<"consumed_id,begin,end,imu_count,imu_fnv64,points,cloud_fnv64\n";
    std::vector<double> v;uint64_t ih=14695981039346656037ULL,ch=ih;
    for(const auto& im:m.imu){
        double a[]={PARITY_SECONDS(im->header.stamp),im->angular_velocity.x,im->angular_velocity.y,im->angular_velocity.z,
                    im->linear_acceleration.x,im->linear_acceleration.y,im->linear_acceleration.z};
        ih=parity_hash(ih,a,sizeof(a));if(parity_active())v.insert(v.end(),a,a+7);
    }
    for(const auto& p:m.lidar->points){float a[]={p.x,p.y,p.z,p.intensity,p.curvature};ch=parity_hash(ch,a,sizeof(a));}
    f<<consumed_lidar_count<<","<<std::setprecision(17)<<m.lidar_beg_time<<","<<m.lidar_end_time<<","<<m.imu.size()<<","<<ih<<","<<m.lidar->size()<<","<<ch<<std::endl;
    if(!f)throw std::runtime_error("parity input log failed");
    parity_values("imu",v);parity_cloud("input",*m.lidar);
}
static void parity_matches(){
    if(!parity_active())return;
    std::vector<double> v;
    for(int i=0;i<feats_down_size;++i){
        v.push_back(i);v.push_back(Nearest_Points[i].size());
        for(const auto& p:Nearest_Points[i]){v.push_back(p.x);v.push_back(p.y);v.push_back(p.z);}
        v.push_back(point_selected_surf[i]?1:0);
        if(point_selected_surf[i]){const auto& p=normvec->points[i];v.push_back(p.x);v.push_back(p.y);v.push_back(p.z);v.push_back(p.intensity);}
    }
    parity_values("iter"+std::to_string(parity_iteration)+"_matches",v);
}
