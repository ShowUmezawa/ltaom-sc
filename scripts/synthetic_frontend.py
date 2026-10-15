#!/usr/bin/env python3
"""Actual native ROS2 messages/services; stationary three-plane frontend fixture."""
import argparse,csv,json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import PointCloud2,PointField,Imu
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
os.environ.setdefault('ROS_DOMAIN_ID','71');os.environ.setdefault('ROS_LOCALHOST_ONLY','1');os.environ['ROS_LOG_DIR']=str(a.out/'ros')
config=yaml.safe_load((Path(__file__).resolve().parents[1]/'src/ltaom_frontend/config/hesai/dog_hesai16.yaml').read_text());config['SaveDir']=str(a.out/'frontend');config['frontend_feedback_en']=False
(a.out/'params.yaml').write_text(yaml.safe_dump({'laserMapping':{'ros__parameters':config}}))
log=(a.out/'frontend.log').open('w');proc=subprocess.Popen([str(Path(os.environ['LTAOM_BUILD_DIR'])/'install/ltaom_frontend/lib/ltaom_frontend/fastlio_node'),'--ros-args','--params-file',str(a.out/'params.yaml')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=rclpy.create_node('synthetic_fixture');cloudpub=node.create_publisher(PointCloud2,'/hesai/pandar',100);imupub=node.create_publisher(Imu,'/imu/body',1000)
odoms=[];sizes=[];subscriptions=[node.create_subscription(Odometry,'/Odometry',lambda m:odoms.append(m),1000),node.create_subscription(PointCloud2,'/cloud_registered',lambda m:sizes.append(m.height*m.width),100)]
def wait(predicate,why,timeout=30):
 end=time.monotonic()+timeout
 while not predicate():
  if proc.poll() is not None:raise RuntimeError('frontend exited '+why)
  if time.monotonic()>end:raise RuntimeError('timeout '+why)
  rclpy.spin_once(node,timeout_sec=.02)
def stamp(t):
 n=int(round(t*1e9));return Time(sec=n//1000000000,nanosec=n%1000000000)
def call(client,request):
 wait(lambda:client.service_is_ready(),'service_ready');f=client.call_async(request);wait(f.done,'service_reply');return f.result()
try:
 wait(lambda:cloudpub.get_subscription_count()>0 and imupub.get_subscription_count()>0,'subscriptions')
 n=6000;rng=np.random.RandomState(7);dtype=np.dtype(dict(names=['x','y','z','intensity','timestamp','ring'],formats=['<f4','<f4','<f4','<f4','<f8','<u2'],offsets=[0,4,8,16,24,32],itemsize=48));data=np.zeros(n,dtype=dtype)
 xyz=rng.uniform(-6,6,(n,3));xyz[:2000,2]=-2;xyz[2000:4000,0]=8;xyz[4000:,1]=9
 for k,name in enumerate(['x','y','z']):data[name]=xyz[:,k]
 data['intensity']=42;data['ring']=np.arange(n)%16
 fields=[PointField(name=name,offset=off,datatype=typ,count=1) for name,off,typ in [('x',0,7),('y',4,7),('z',8,7),('intensity',16,7),('timestamp',24,8),('ring',32,4)]]
 bad=PointCloud2();bad.header.stamp=stamp(1781091000.0);cloudpub.publish(bad)
 for frame in range(35):
  t=1781091000+frame*.1
  for k in range(20):
   m=Imu();m.header.stamp=stamp(t+k*.005);m.linear_acceleration.z=9.81;imupub.publish(m)
  data['timestamp']=t+np.linspace(0,.1,n);m=PointCloud2();m.header.stamp=stamp(t);m.header.frame_id='hesai';m.height=1;m.width=n;m.fields=fields;m.point_step=48;m.row_step=n*48;m.is_dense=True;m.data=data.tobytes();cloudpub.publish(m)
  end=time.monotonic()+.08
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
 for k in range(22):
  m=Imu();m.header.stamp=stamp(1781091003.5+k*.005);m.linear_acceleration.z=9.81;imupub.publish(m)
 params=node.create_client(SetParameters,'/laserMapping/set_parameters');wait(params.service_is_ready,'parameter_service')
 request=SetParameters.Request();request.parameters=[p.to_parameter_msg() for p in [Parameter('expected_lidar_messages',value=36),Parameter('expected_imu_messages',value=722)]];future=params.call_async(request);wait(future.done,'set_expected_counts');assert all(r.successful for r in future.result().results)
 client=node.create_client(Trigger,'/frontend/drain');wait(lambda:call(client,Trigger.Request()).success,'frontend_eof',60)
 status=json.loads((a.out/'frontend/frontend_status.json').read_text());assert status['received_lidar']==36 and status['received_imu']==722 and status['invalid_lidar']==1 and status['consumed_lidar']==35 and status['uncovered_tail']==0
 wait(lambda:len(odoms)==status['output_scans'] and len(sizes)==len(odoms),'output_counts')
 rows=list(csv.DictReader((a.out/'frontend/scan_manifest.csv').open()));assert len(rows)>=10
 starts=np.array([float(r['association_time']) for r in rows]);ends=np.array([float(r['state_time']) for r in rows]);ext=np.array([[float(r[k]) for k in ['ext_tx','ext_ty','ext_tz','ext_qx','ext_qy','ext_qz','ext_qw']] for r in rows]);tum=np.loadtxt(a.out/'frontend/trajectory.tum')
 assert np.isfinite(tum).all() and np.max(np.abs((ends-starts)-.1))<2e-6 and np.max(np.abs(ext-ext[0]))<1e-10
 assert np.max(np.abs(tum[:,0]-ends))<2e-6 and np.max(np.diff(ends))<.101 and abs(ends[-1]-1781091003.5)<2e-6 and max(sizes)<=6000
 (a.out/'result.json').write_text(json.dumps(dict(status='passed',kind='ROS2 native frontend synthetic only; not real-data accuracy',frontend=status,outputs=len(rows),fixed_extrinsic_max_change=float(np.max(np.abs(ext-ext[0]))),last_state_time=float(ends[-1]),max_cloud_points=max(sizes)),indent=2)+'\n')
finally:
 node.destroy_node();rclpy.shutdown()
 if proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=20)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 log.close()

text=(a.out/'frontend.log').read_text()
if proc.returncode!=0 or 'Error in destruction' in text or 'Failed to delete data' in text:
 (a.out/'result.json').write_text(json.dumps(dict(status='failed',reason='frontend_shutdown',returncode=proc.returncode),indent=2)+'\n')
 raise RuntimeError('unclean frontend shutdown')
print('ROS2_SYNTHETIC_FRONTEND_PASSED',flush=True)
