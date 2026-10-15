#pragma once
#include <ltaom_interfaces/msg/submap.hpp>
#include <openssl/sha.h>
#include <array>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
namespace ltaom_protocol {
// Canonical schema 1: little-endian scalars, length-prefixed UTF-8 strings,
// row-major XYZI float32 payload. PCL padding and field offsets are excluded.
inline std::string submapHash(const ltaom_interfaces::msg::Submap& m) {
 SHA256_CTX ctx;SHA256_Init(&ctx);
 auto bytes=[&](const void* p,size_t n){SHA256_Update(&ctx,p,n);};
 auto integer=[&](uint64_t v,size_t n){uint8_t b[8];for(size_t i=0;i<n;++i)b[i]=uint8_t(v>>(8*i));bytes(b,n);};
 auto string=[&](const std::string& s){integer(s.size(),4);bytes(s.data(),s.size());};
 auto stamp=[&](const builtin_interfaces::msg::Time& t){integer(uint32_t(t.sec),4);integer(t.nanosec,4);};
 auto real=[&](double v){uint64_t bits;std::memcpy(&bits,&v,8);integer(bits,8);};
 const std::string tag="LTAOM_SUBMAP_XYZI_V1";bytes(tag.data(),tag.size());
 integer(m.schema,4);string(m.session_id);integer(m.submap_id,8);integer(m.first_scan_id,8);integer(m.last_scan_id,8);stamp(m.association_time);
 const auto& p=m.anchor_pose;for(double v:{p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w})real(v);
 const auto& c=m.cloud;stamp(c.header.stamp);string(c.header.frame_id);integer(c.height,4);integer(c.width,4);integer(c.is_dense?1:0,1);
 if(c.is_bigendian||!c.width||!c.height||c.fields.size()!=4||c.row_step<uint64_t(c.width)*c.point_step||c.data.size()!=uint64_t(c.row_step)*c.height)throw std::runtime_error("invalid XYZI cloud layout");
 std::array<uint32_t,4> offsets;const char* names[]={"x","y","z","intensity"};
 for(int i=0;i<4;++i){int matches=0;for(const auto& f:c.fields)if(f.name==names[i]){if(f.datatype!=7||f.count!=1||uint64_t(f.offset)+4>c.point_step)throw std::runtime_error("invalid XYZI field");offsets[i]=f.offset;++matches;}if(matches!=1)throw std::runtime_error("missing or duplicate XYZI field");}
 for(uint32_t row=0;row<c.height;++row)for(uint32_t col=0;col<c.width;++col)for(auto offset:offsets)bytes(c.data.data()+size_t(row)*c.row_step+size_t(col)*c.point_step+offset,4);
 unsigned char digest[SHA256_DIGEST_LENGTH];SHA256_Final(digest,&ctx);std::ostringstream out;out<<std::hex<<std::setfill('0');for(auto b:digest)out<<std::setw(2)<<int(b);return out.str();
}
}
