"""Configuration and unit contracts; run with python3 -m unittest discover -s tests."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from sensor_profile import (load_profile, frontend_parameters, convert_imu,
                            check_bag_topics, snapshot_profile)
ROS = 'ros1' if (ROOT / 'src/fast_lio').exists() else 'ros2'


class SensorProfileTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.base = load_profile(ROOT, ROS, 'dog', full=True)
        self.front = yaml.safe_load(Path(self.base['frontend']).read_text())
        (self.root/'frontend.yaml').write_text(yaml.safe_dump(self.front))
        self.data = dict(schema=1, name='test_sensor', topics=dict(lidar='/custom/cloud', imu='/custom/imu'),
                         imu_units=dict(acceleration='g', angular_velocity='deg/s'), frontend='frontend.yaml',
                         std=self.base['std'], geometry=self.base['geometry'])
        self.path = self.root/'sensor.yaml'

    def load(self, full=False):
        self.path.write_text(yaml.safe_dump(self.data))
        return load_profile(ROOT, ROS, sensor_config=self.path, full=full)

    def test_presets_keep_topics_scales_and_frontend_numbers(self):
        for preset in ('car', 'dog'):
            p = load_profile(ROOT, ROS, preset, full=True)
            actual = frontend_parameters(p, '/output')
            expected = yaml.safe_load(Path(p['frontend']).read_text())
            expected.update(SaveDir='/output', frontend_feedback_en=False, multisession_mode=0, ikdtree_rebuild_mode='sync')
            expected['common'].update(lid_topic=p['lidar'], imu_topic=p['imu'])
            self.assertEqual(actual, expected)
            self.assertEqual(p['acceleration_scale'], 9.80665 if preset=='car' else 1)
            self.assertFalse(p['scale_covariances'])
        self.assertEqual(load_profile(ROOT, ROS, 'car')['imu'], '/imu/si')
        self.assertEqual(load_profile(ROOT, ROS, 'dog')['bag_imu'],
                         '/imu/body' if ROS == 'ros1' else '/go2_research/body_imu/host')

    def test_paths_topics_snapshot_and_no_relay_for_si(self):
        p = self.load(full=True)
        self.assertEqual(p['frontend'],str(self.root/'frontend.yaml'))
        self.assertEqual(p['imu'],'/custom/imu_si')
        out=self.root/'out';out.mkdir();snapshot_profile(p,out)
        self.assertEqual((out/'input_frontend.yaml').read_bytes(),(self.root/'frontend.yaml').read_bytes())
        self.assertIn('frontend',json.loads((out/'sensor_profile.json').read_text())['config_sha256'])
        self.data['imu_units']=dict(acceleration='m/s2',angular_velocity='rad/s')
        p=self.load();self.assertFalse(p['relay']);self.assertEqual(p['imu'],p['bag_imu'])

    def test_explicit_units_and_topic_safety(self):
        cases=[('units',lambda d:d['imu_units'].update(acceleration='meters')),
               ('missing',lambda d:d.pop('imu_units')),
               ('loop',lambda d:d['topics'].update(imu_output='/custom/imu')),
               ('same',lambda d:d['topics'].update(lidar='/custom/imu')),
               ('relative',lambda d:d['topics'].update(lidar='cloud')),
               ('nonstring',lambda d:d['topics'].update(imu=12)),
               ('typo',lambda d:d.update(fronted='frontend.yaml'))]
        for _,change in cases:
            original=copy.deepcopy(self.data);change(self.data)
            with self.assertRaises(ValueError):self.load()
            self.data=original

    def test_extrinsics_and_time_validation(self):
        cases=[lambda f:f['mapping'].update(extrinsic_R=None),
               lambda f:f['mapping'].update(extrinsic_T=[0,0,float('nan')]),
               lambda f:f['mapping'].update(extrinsic_R=[1,0,0,0,1,0,0,0,-1]),
               lambda f:f['common'].update(time_offset_from_lidar=1,time_offset_lidar_to_imu=2),
               lambda f:f['preprocess'].update(scan_line=0),
               lambda f:f.update(point_filter_num=0)]
        for change in cases:
            f=copy.deepcopy(self.front);change(f)
            (self.root/'frontend.yaml').write_text(yaml.safe_dump(f))
            with self.assertRaises(ValueError):self.load()

    def test_full_pipeline_requires_configs(self):
        del self.data['std'];del self.data['geometry']
        self.load()
        with self.assertRaises(ValueError):self.load(full=True)

    def test_nested_override_preserves_time_and_reserves_runner_fields(self):
        p=self.load();patch=self.root/'override.json'
        patch.write_text(json.dumps({'mapping':{'acc_cov':0.25},'common':{'time_offset_lidar_to_imu':0.02}}))
        c=frontend_parameters(p,'/output',override=patch)
        self.assertEqual(c['mapping']['extrinsic_R'],self.front['mapping']['extrinsic_R'])
        self.assertEqual(c['mapping']['acc_cov'],0.25)
        self.assertEqual(c['common']['lid_topic'],'/custom/cloud')
        for d in [{'common':{'imu_topic':'/wrong'}},{'SaveDir':'/wrong'},{'frontend_feedback_en':True},{'multisession_mode':1}]:
            patch.write_text(json.dumps(d))
            with self.assertRaises(ValueError):frontend_parameters(p,'/output',override=patch)

    def test_bag_types(self):
        p=self.load()
        topics={'/custom/cloud':'sensor_msgs/PointCloud2','/custom/imu':'sensor_msgs/Imu'}
        check_bag_topics(p,topics,'ros1')
        check_bag_topics(p,{k:v.replace('sensor_msgs/','sensor_msgs/msg/') for k,v in topics.items()},'ros2')
        with self.assertRaises(ValueError):check_bag_topics(p,topics,'ros2')
        with self.assertRaises(ValueError):check_bag_topics(p,{},'ros1')

    def test_unit_conversion_and_covariance_sentinel(self):
        p=self.load()
        m=SimpleNamespace(header=object(),orientation=object(),linear_acceleration=SimpleNamespace(x=1.,y=2.,z=-1.),
                          angular_velocity=SimpleNamespace(x=180.,y=0.,z=-90.),
                          linear_acceleration_covariance=[.2]*9,angular_velocity_covariance=[.3]*9)
        header=m.header;orientation=m.orientation
        convert_imu(m,p)
        self.assertAlmostEqual(m.linear_acceleration.z,-9.80665)
        self.assertAlmostEqual(m.angular_velocity.x,math.pi)
        self.assertAlmostEqual(m.linear_acceleration_covariance[4],.2*9.80665**2)
        self.assertAlmostEqual(m.angular_velocity_covariance[4],.3*(math.pi/180)**2)
        self.assertIs(m.header,header);self.assertIs(m.orientation,orientation)
        m.linear_acceleration_covariance=[-1.]+[0.]*8
        m.angular_velocity_covariance=[0.]*9
        convert_imu(m,p)
        self.assertEqual(m.linear_acceleration_covariance,[-1.]+[0.]*8)
        self.assertEqual(m.angular_velocity_covariance,[0.]*9)

    def test_template_requires_calibration(self):
        with self.assertRaisesRegex(ValueError,'extrinsic'):
            load_profile(ROOT,ROS,sensor_config=ROOT/'config/sensors/custom.example.yaml')

    def test_presets_and_custom_are_mutually_exclusive(self):
        self.path.write_text(yaml.safe_dump(self.data))
        with self.assertRaises(ValueError):load_profile(ROOT,ROS,'car',self.path)


if __name__=='__main__':unittest.main()
