#pragma once
// Diagnostic frontend filter: use a total order (voxel key, input index)
// before the original PCL float centroid accumulation. No centroid reweighting.
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>
template<class PointT>
void orderedVoxelFilter(const pcl::PointCloud<PointT>& input,
                        pcl::PointCloud<PointT>& output, float leaf) {
  if(!std::isfinite(leaf)||leaf<=0)throw std::invalid_argument("ordered voxel: invalid leaf");
  output.clear();output.header=input.header;
  output.sensor_origin_=input.sensor_origin_;output.sensor_orientation_=input.sensor_orientation_;
  output.height=1;output.is_dense=true;
  if(input.empty()){output.width=0;return;}
  // Guard integer conversion even if upstream marked a cloud dense incorrectly.
  for(const auto& p:input.points)
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))
      throw std::invalid_argument("ordered voxel: nonfinite XYZ");
  Eigen::Vector4f lo,hi;pcl::getMinMax3D(input,lo,hi);
  const float inv=1.0f/leaf;
  Eigen::Vector3i minb,maxb,div;
  std::int64_t volume=1;
  for(int k=0;k<3;++k){
    const double l=std::floor(lo[k]*inv),h=std::floor(hi[k]*inv);
    if(l<std::numeric_limits<int>::min()||h>std::numeric_limits<int>::max()||!std::isfinite(l)||!std::isfinite(h))
      throw std::overflow_error("ordered voxel: coordinate index overflow");
    minb[k]=static_cast<int>(l);maxb[k]=static_cast<int>(h);
    const std::int64_t size=std::int64_t(maxb[k])-minb[k]+1;
    if(size<=0||size>std::numeric_limits<int>::max()/volume)
      throw std::overflow_error("ordered voxel: grid index overflow");
    div[k]=static_cast<int>(size);volume*=size;
  }
  struct Entry { unsigned key;size_t point; };
  std::vector<Entry> order;order.reserve(input.size());
  for(size_t i=0;i<input.size();++i){
    const auto& p=input[i];
    // Preserve PCL's float subtraction and voxel key arithmetic.
    int x=static_cast<int>(std::floor(p.x*inv)-static_cast<float>(minb[0]));
    int y=static_cast<int>(std::floor(p.y*inv)-static_cast<float>(minb[1]));
    int z=static_cast<int>(std::floor(p.z*inv)-static_cast<float>(minb[2]));
    const std::int64_t key=std::int64_t(x)+std::int64_t(y)*div[0]+std::int64_t(z)*div[0]*div[1];
    if(key<0||key>=volume)throw std::overflow_error("ordered voxel: invalid key");
    order.push_back({static_cast<unsigned>(key),i});
  }
  std::sort(order.begin(),order.end(),[](const Entry& a,const Entry& b){return a.key<b.key||(a.key==b.key&&a.point<b.point);});
  output.reserve(input.size());
  for(size_t i=0;i<order.size();){
    size_t j=i;pcl::CentroidPoint<PointT> centroid;
    while(j<order.size()&&order[j].key==order[i].key)centroid.add(input[order[j++].point]);
    PointT point{};centroid.get(point);output.push_back(point);i=j;
  }
  output.width=static_cast<std::uint32_t>(output.size());
}
