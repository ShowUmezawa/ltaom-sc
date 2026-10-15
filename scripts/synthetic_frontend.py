#!/usr/bin/env python3
"""Synthetic stationary three-plane fixture. Does not read datasets or bags."""
import argparse,csv,json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2,PointField,Imu
from nav_msgs.msg import Odometry

parser=argparse.ArgumentParser();parser.add_argument('--platform',choices=['car','dog'],required=True);parser.add_argument('--out',required=True)
args=parser.parse_args();out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11431';os.environ['ROS_HOSTNAME']='127.0.0.1';os.environ['ROS_LOG_DIR']=str(out/'ros')
logs=[];processes=[]
def start(cmd,name):
 log=open(out/name,'w');logs.append(log);p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
try:
 core=start(['roscore','-p','11431'],'roscore.log')
 for _ in range(50):
  if subprocess.run(['rosparam','list'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0:break
  time.sleep(.1)
 else:raise RuntimeError('roscore did not start')
 launch=start(['roslaunch','fast_lio','frontend_only.launch','platform:='+args.platform,'output_dir:='+str(out/'frontend')],'launch.log')
 rospy.init_node('synthetic_fixture',anonymous=True)
 cloudpub=rospy.Publisher('/hesai/pandar',PointCloud2,queue_size=20)
 imupub=rospy.Publisher('/imu/data' if args.platform=='car' else '/imu/body',Imu,queue_size=1000)
 odoms=[];cloudsizes=[]
 rospy.Subscriber('/Odometry',Odometry,lambda m:odoms.append(m.header.stamp.to_sec()),queue_size=1000)
 rospy.Subscriber('/cloud_registered',PointCloud2,lambda m:cloudsizes.append(m.width*m.height),queue_size=100)
 for _ in range(100):
  if cloudpub.get_num_connections() and imupub.get_num_connections():break
  if launch.poll() is not None:raise RuntimeError('launch exited before sensor subscription')
  time.sleep(.1)
 else:raise RuntimeError('subscriptions did not connect')
 # Empty input must not desynchronize the next valid scan.
 bad=PointCloud2();bad.header.stamp=rospy.Time.from_sec(1781091000.0);cloudpub.publish(bad)
 n=6000; rng=np.random.RandomState(7)
 dtype=np.dtype({'names':['x','y','z','intensity','timestamp','ring'],
 'formats':['<f4','<f4','<f4','<f4','<f8','<u2'],'offsets':[0,4,8,16,24,32],'itemsize':48})
 data=np.zeros(n,dtype=dtype)
 xyz=rng.uniform(-6,6,(n,3));xyz[:2000,2]=-2;xyz[2000:4000,0]=8;xyz[4000:,1]=9
 for k,name in enumerate(['x','y','z']):data[name]=xyz[:,k]
 data['intensity']=42;data['ring']=np.arange(n)%(32 if args.platform=='car' else 16)
 fields=[PointField(name,offset,typ,1) for name,offset,typ in [('x',0,7),('y',4,7),('z',8,7),('intensity',16,7),('timestamp',24,8),('ring',32,4)]]
 offset=-36.89 if args.platform=='car' else 0.
 gravity=1. if args.platform=='car' else 9.81
 for frame in range(35):
  t=1781091000.0+frame*.1
  for k in range(20):
   m=Imu();m.header.stamp=rospy.Time.from_sec(t+k*.005+offset);m.linear_acceleration.z=gravity;imupub.publish(m)
  data['timestamp']=t+np.linspace(0,.1,n)
  cloud=PointCloud2();cloud.header.stamp=rospy.Time.from_sec(t);cloud.header.frame_id='hesai'
  cloud.height=1;cloud.width=n;cloud.fields=fields;cloud.point_step=48;cloud.row_step=n*48;cloud.is_dense=True;cloud.data=data.tobytes()
  cloudpub.publish(cloud);time.sleep(.08)
 # Cover the final scan end; no bag is involved.
 for k in range(22):
  m=Imu();m.header.stamp=rospy.Time.from_sec(1781091003.5+k*.005+offset);m.linear_acceleration.z=gravity;imupub.publish(m)
 for _ in range(100):
  if odoms and odoms[-1]>=1781091003.5-1e-5:break
  if launch.poll() is not None:raise RuntimeError('frontend exited')
  time.sleep(.1)
 rows=list(csv.DictReader(open(out/'frontend/scan_manifest.csv')))
 if len(rows)<10:raise RuntimeError('too few output scans: '+str(len(rows)))
 starts=np.array([float(r['association_time']) for r in rows]);ends=np.array([float(r['state_time']) for r in rows])
 ext=np.array([[float(r[k]) for k in ['ext_tx','ext_ty','ext_tz','ext_qx','ext_qy','ext_qz','ext_qw']] for r in rows])
 tum=np.loadtxt(out/'frontend/trajectory.tum')
 assert np.isfinite(tum).all() and np.isfinite(ext).all()
 assert np.max(np.abs((ends-starts)-.1))<2e-6,'wrong scan end time'
 assert np.max(np.abs(ext-ext[0]))<1e-10,'fixed extrinsics changed'
 assert np.max(np.abs(tum[:,0]-ends))<2e-6,'TUM not stamped at state time'
 assert np.max(np.diff(ends))<.101,'internal frame gap'
 assert abs(ends[-1]-1781091003.5)<2e-6,'last frame not drained'
 assert cloudsizes and max(cloudsizes)<=6000,'spurious points in first cloud'
 result={'platform':args.platform,'kind':'synthetic fixture; no dataset run','poses':len(rows),'scan_duration_s':float(np.mean(ends-starts)),
 'fixed_extrinsic_max_change':float(np.max(np.abs(ext-ext[0]))),'max_cloud_points':max(cloudsizes),'status':'passed'}
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
 subprocess.run(['rosparam','dump',str(out/'parameters.yaml')],check=True)
finally:
 for p in reversed(processes):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=10)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 for log in logs:log.close()
