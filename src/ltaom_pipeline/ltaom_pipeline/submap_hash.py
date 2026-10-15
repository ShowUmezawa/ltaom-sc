"""Canonical digest shared with ltaom_transport/submap_hash.hpp, no CDR padding."""
import hashlib,struct
import numpy as np
def cloud_xyzi(c):
 if c.is_bigendian or not c.height or not c.width or len(c.fields)!=4 or c.row_step<c.width*c.point_step or len(c.data)!=c.row_step*c.height:raise ValueError('invalid XYZI layout')
 fields={f.name:f for f in c.fields};names=['x','y','z','intensity']
 if len(fields)!=4 or any(n not in fields or fields[n].datatype!=7 or fields[n].count!=1 or fields[n].offset+4>c.point_step for n in names):raise ValueError('invalid XYZI fields')
 dtype=np.dtype(dict(names=names,formats=['<f4']*4,offsets=[fields[n].offset for n in names],itemsize=c.point_step))
 a=np.ndarray((c.height,c.width),dtype=dtype,buffer=c.data,strides=(c.row_step,c.point_step))
 return np.stack([a[n] for n in names],axis=-1).reshape(-1,4).astype('<f4')
def submap_hash(m):
 h=hashlib.sha256(b'LTAOM_SUBMAP_XYZI_V1')
 def string(s):
  b=s.encode();h.update(struct.pack('<I',len(b)));h.update(b)
 def stamp(t):h.update(struct.pack('<iI',t.sec,t.nanosec))
 h.update(struct.pack('<I',m.schema));string(m.session_id);h.update(struct.pack('<QQQ',m.submap_id,m.first_scan_id,m.last_scan_id));stamp(m.association_time)
 p=m.anchor_pose;h.update(struct.pack('<7d',p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w))
 c=m.cloud;stamp(c.header.stamp);string(c.header.frame_id);h.update(struct.pack('<IIB',c.height,c.width,c.is_dense));h.update(cloud_xyzi(c).tobytes())
 return h.hexdigest()
