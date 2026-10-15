"""Atomic ROS2 Submap -> immutable local/world PCDs and explicit identity record."""
import csv,json,math,re
import numpy as np
from .pipeline_core import dump,sha,matrix,write_pcd
from .submap_hash import cloud_xyzi,submap_hash
class Archive:
 def __init__(self,root,session,scan_manifest):
  if not re.fullmatch(r'[A-Za-z0-9_-]+',session):raise ValueError('invalid session')
  self.root,self.session,self.scan_manifest=root,session,scan_manifest
  root.mkdir(parents=True,exist_ok=False);(root/'clouds').mkdir();self.stream=(root/'submaps.jsonl').open('x');self.records=[]
 def add(self,m):
  if m.schema!=1 or m.session_id!=self.session or m.submap_id!=len(self.records):raise ValueError('submap identity')
  digest=submap_hash(m)
  if digest!=m.semantic_sha256:raise ValueError('submap digest mismatch')
  if m.association_time!=m.cloud.header.stamp:raise ValueError('submap stamp mismatch')
  scans=list(csv.DictReader(self.scan_manifest.open()));first,last=m.first_scan_id,m.last_scan_id
  if first>last or last>=len(scans) or first!=(self.records[-1]['last_scan_id']+1 if self.records else 0):raise ValueError('submap scan coverage')
  for i in [first,last]:
   if int(scans[i]['scan_id'])!=i:raise ValueError('scan manifest sequence')
  stamp=m.association_time.sec*10**9+m.association_time.nanosec
  if abs(float(scans[last]['association_time'])-stamp/1e9)>2e-6:raise ValueError('scan manifest association mismatch')
  time=float(scans[last]['state_time'])
  if not math.isfinite(time) or (self.records and time<=self.records[-1]['state_time']):raise ValueError('nonmonotonic state time')
  p,q=m.anchor_pose.position,m.anchor_pose.orientation;pose=[p.x,p.y,p.z,q.x,q.y,q.z,q.w];x=matrix(pose)
  world=cloud_xyzi(m.cloud)
  if not np.isfinite(world).all():raise ValueError('nonfinite cloud')
  local=world.astype(float);local[:,:3]=(local[:,:3]-x[:3,3])@x[:3,:3];local=local.astype('<f4')
  error=float(np.linalg.norm(local[:,:3].astype(float)@x[:3,:3].T+x[:3,3]-world[:,:3],axis=1).max())
  if error>.001:raise ValueError('local/world roundtrip exceeds 1 mm')
  yaw=math.atan2(x[1,0],x[0,0]);c,s=math.cos(yaw),math.sin(yaw);gravity=np.array([[c,s,0],[-s,c,0],[0,0,1]])@x[:3,:3]
  i=m.submap_id;lp='clouds/%06d.local.pcd'%i;wp='clouds/%06d.world.pcd'%i
  write_pcd(self.root/lp,local);write_pcd(self.root/wp,world)
  r=dict(schema=1,session_id=self.session,submap_id=i,association_time_ns=stamp,state_time=time,first_scan_id=first,last_scan_id=last,raw_anchor_pose=pose,
   pose_convention='T_W_B; xyz,qx,qy,qz,qw',world_frame=m.cloud.header.frame_id,local_frame='B_%d'%i,cloud_path=lp,cloud_sha256=sha(self.root/lp),world_cloud_path=wp,world_cloud_sha256=sha(self.root/wp),points=len(world),transport_semantic_sha256=digest,transport_hash_schema='LTAOM_SUBMAP_XYZI_V1',gravity_alignment_R=gravity.ravel().tolist(),gravity_direction_local=(-x[2,:3]).tolist(),gravity_quality='frontend_world_z_assumed_unvalidated',local_world_roundtrip_max_m=error,extrinsics='already_applied_by_frontend')
  self.stream.write(json.dumps(r,allow_nan=False)+'\n');self.stream.flush();self.records.append(r);return r
 def finish(self):
  self.stream.close();dump(self.root/'manifest.json',dict(schema=1,status='complete',session_id=self.session,submaps=len(self.records),submaps_sha256=sha(self.root/'submaps.jsonl'),scan_manifest_sha256=sha(self.scan_manifest),immutable=True,max_roundtrip_m=max(r['local_world_roundtrip_max_m'] for r in self.records)))
