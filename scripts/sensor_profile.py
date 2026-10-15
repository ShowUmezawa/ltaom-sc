"""Sensor profiles shared by ROS1 and ROS2 replay runners (no ROS imports)."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np
import yaml


def add_profile_arguments(parser, required=False):
    group = parser.add_mutually_exclusive_group(required=required)
    group.add_argument('--dataset', choices=['car', 'dog'], help='Built-in sensor preset')
    group.add_argument('--sensor-config', type=Path, help='Custom sensor profile YAML')


def _keys(value, allowed, name):
    if not isinstance(value, dict):
        raise ValueError(name + ' must be a mapping')
    unknown = set(value) - set(allowed)
    if unknown:
        raise ValueError(name + ': unknown keys ' + ', '.join(sorted(unknown)))


def _number(value, name, minimum=None, integer=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name + ' must be a finite number')
    if integer and type(value) is not int:
        raise ValueError(name + ' must be an integer')
    if minimum is not None and value < minimum:
        raise ValueError(name + ' must be >= ' + str(minimum))
    return value


def _file(base, value, name):
    if not isinstance(value, str) or not value:
        raise ValueError(name + ' must name a configuration file')
    path = (base / value).resolve()
    if not path.is_file():
        raise ValueError(name + ' does not exist: ' + str(path))
    return str(path)


def load_profile(repo, ros, dataset=None, sensor_config=None, full=False):
    repo = Path(repo)
    if dataset and sensor_config:
        raise ValueError('--dataset and --sensor-config are mutually exclusive')
    if sensor_config is None:
        dataset = dataset or 'dog'
        if dataset not in ('car', 'dog'):
            raise ValueError('unknown dataset')
        car = dataset == 'car'
        front = repo / ('src/fast_lio' if ros == 'ros1' else 'src/ltaom_frontend') / 'config/hesai'
        if ros == 'ros1':
            std = repo / ('src/STD/config/config_velo_mulran.yaml' if car else 'config/dog_std.yaml')
            geom = repo / 'src/loop_pipeline/config'
            lidar, raw_imu = '/hesai/pandar', '/imu/data' if car else '/imu/body'
        else:
            std = repo / 'src/ltaom_std/config' / ('config_velo_mulran.yaml' if car else 'dog_std.yaml')
            geom = repo / 'src/ltaom_pipeline/config'
            lidar = '/hesai/pandar' if car else '/go2_research/hesai/points'
            raw_imu = '/imu/data' if car else '/go2_research/body_imu/host'
        result = dict(name=dataset, description=dataset + ' sensor preset', lidar=lidar,
                      bag_imu=raw_imu, imu='/imu/si' if car else raw_imu,
                      frontend=str((front / ('car_hesai32.yaml' if car else 'dog_hesai16.yaml')).resolve()),
                      std=str(std.resolve()), geometry=str((geom / (dataset + '_geometry.json')).resolve()),
                      acceleration_scale=9.80665 if car else 1.0, angular_velocity_scale=1.0,
                      scale_covariances=False, gap=50 if car else 8, seconds=30 if car else 15,
                      top_k=10, sc_threshold=0.13, custom=False)
    else:
        path = Path(sensor_config).resolve()
        data = yaml.safe_load(path.read_text())
        _keys(data, ['schema', 'name', 'topics', 'imu_units', 'frontend', 'std', 'geometry', 'retrieval'], 'profile')
        if type(data.get('schema')) is not int or data['schema'] != 1:
            raise ValueError('profile.schema must be 1')
        name = data.get('name')
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', name):
            raise ValueError('profile.name must contain letters, digits, underscores or hyphens')
        topics = data.get('topics')
        _keys(topics, ['lidar', 'imu', 'imu_output'], 'topics')
        for key in ('lidar', 'imu'):
            if not isinstance(topics.get(key), str):
                raise ValueError('topics.' + key + ' must be a string')
        units = data.get('imu_units')
        _keys(units, ['acceleration', 'angular_velocity'], 'imu_units')
        if units.get('acceleration') not in ('m/s2', 'g') or units.get('angular_velocity') not in ('rad/s', 'deg/s'):
            raise ValueError('imu_units must specify acceleration: m/s2|g and angular_velocity: rad/s|deg/s')
        acc = 9.80665 if units['acceleration'] == 'g' else 1.0
        gyro = math.pi / 180.0 if units['angular_velocity'] == 'deg/s' else 1.0
        relay = acc != 1.0 or gyro != 1.0
        output = topics.get('imu_output', topics['imu'] + '_si' if relay else topics['imu'])
        for key, topic in dict(lidar=topics['lidar'], imu=topics['imu'], imu_output=output).items():
            if not isinstance(topic, str) or not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', topic):
                raise ValueError('topics.' + key + ' must be an absolute ROS topic name')
        if topics['lidar'] in (topics['imu'], output):
            raise ValueError('LiDAR and IMU topics must differ')
        if relay and output == topics['imu']:
            raise ValueError('unit conversion requires a separate imu_output topic')
        if not relay and output != topics['imu']:
            raise ValueError('SI input uses its original topic; omit imu_output')
        retrieval = data.get('retrieval', {})
        _keys(retrieval, ['gap', 'seconds', 'top_k', 'sc_threshold'], 'retrieval')
        values = dict(gap=50, seconds=30.0, top_k=10, sc_threshold=0.13)
        values.update(retrieval)
        for key in values:
            _number(values[key], 'retrieval.' + key, 1 if key in ('gap', 'top_k') else 0, key in ('gap', 'top_k'))
        if values['sc_threshold'] > 2:
            raise ValueError('retrieval.sc_threshold must be <= 2')
        result = dict(name=name, description=name + ' custom sensor profile', custom=True,
                      lidar=topics['lidar'], bag_imu=topics['imu'], imu=output,
                      acceleration_scale=acc, angular_velocity_scale=gyro, scale_covariances=True,
                      frontend=_file(path.parent, data.get('frontend'), 'frontend'), profile_path=str(path), **values)
        for key in ('std', 'geometry'):
            result[key] = _file(path.parent, data.get(key), key) if full or key in data else None
    config = yaml.safe_load(Path(result['frontend']).read_text())
    validate_frontend(config)
    result['relay'] = result['acceleration_scale'] != 1.0 or result['angular_velocity_scale'] != 1.0
    if full:
        geometry = json.loads(Path(result['geometry']).read_text())
        if not isinstance(geometry, dict):
            raise ValueError('geometry must be a JSON object')
        if not Path(result['std']).is_file():
            raise ValueError('STD configuration is missing')
    return result


def validate_frontend(config):
    if not isinstance(config, dict):
        raise ValueError('frontend must be a plain parameter mapping, without ros__parameters wrapper')
    for name in ('common', 'mapping', 'preprocess'):
        if not isinstance(config.get(name), dict):
            raise ValueError('frontend is missing ' + name)
    pre, mapping, common = config['preprocess'], config['mapping'], config['common']
    if pre.get('lidar_type') != 6:
        raise ValueError('sensor profiles currently support the Hesai PointCloud2 adapter (lidar_type: 6)')
    lines = _number(pre.get('scan_line'), 'preprocess.scan_line', 1, True)
    if lines > 128:
        raise ValueError('preprocess.scan_line must be <= 128')
    if config.get('feature_extract_enable', False) is not False:
        raise ValueError('Hesai adapter requires feature_extract_enable: false')
    _number(config.get('point_filter_num', 1), 'point_filter_num', 1, True)
    _number(pre.get('blind', 0.01), 'preprocess.blind', 0)
    for name in ('max_scan_duration_s', 'header_time_tolerance_s'):
        _number(pre.get(name, 0.2 if name == 'max_scan_duration_s' else 0.002), 'preprocess.' + name, 1e-12)
    for name in ('acc_cov', 'gyr_cov', 'b_acc_cov', 'b_gyr_cov'):
        _number(mapping.get(name), 'mapping.' + name, 0)
    R = np.asarray(mapping.get('extrinsic_R'), dtype=float)
    t = np.asarray(mapping.get('extrinsic_T'), dtype=float)
    if R.shape != (9,) or t.shape != (3,) or not np.isfinite(R).all() or not np.isfinite(t).all():
        raise ValueError('set calibrated mapping.extrinsic_R (9 row-major values) and extrinsic_T (3 metres)')
    R = R.reshape(3, 3)
    if np.max(np.abs(R.T @ R - np.eye(3))) > 1e-3 or abs(np.linalg.det(R) - 1.0) > 1e-3:
        raise ValueError('mapping.extrinsic_R must be an orthonormal rotation with determinant +1')
    offsets = [_number(common.get(k, 0.0), 'common.' + k) for k in ('time_offset_lidar_to_imu', 'time_offset_from_lidar')]
    if all(v != 0 for v in offsets):
        raise ValueError('set at most one time offset; do not compensate both clocks')


def _merge(base, patch):
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = copy.deepcopy(value)


def frontend_parameters(profile, output, rebuild_mode='sync', override=None):
    config = yaml.safe_load(Path(profile['frontend']).read_text())
    if override:
        patch = json.loads(Path(override).read_text())
        if not isinstance(patch, dict):
            raise ValueError('--params-override must contain a JSON object')
        reserved = {'SaveDir', 'frontend_feedback_en', 'multisession_mode', 'ikdtree_rebuild_mode',
                    'expected_lidar_messages', 'expected_imu_messages'}
        if reserved.intersection(patch) or any(k in patch for k in ('common.lid_topic', 'common.imu_topic', 'common/lid_topic', 'common/imu_topic')):
            raise ValueError('--params-override cannot change runner-owned parameters')
        if 'common' in patch and (not isinstance(patch['common'], dict) or {'lid_topic', 'imu_topic'}.intersection(patch['common'])):
            raise ValueError('set topics in --sensor-config, not --params-override')
        _merge(config, patch)
    validate_frontend(config)
    config.update(SaveDir=str(output), frontend_feedback_en=False, multisession_mode=0, ikdtree_rebuild_mode=rebuild_mode)
    config['common'].update(lid_topic=profile['lidar'], imu_topic=profile['imu'])
    return config


def check_bag_topics(profile, topics, ros):
    expected = [(profile['lidar'], 'sensor_msgs/PointCloud2'), (profile['bag_imu'], 'sensor_msgs/Imu')]
    for name, typ in expected:
        if ros == 'ros2':
            typ = typ.replace('sensor_msgs/', 'sensor_msgs/msg/')
        if name not in topics or topics[name] != typ:
            raise ValueError('bag must contain ' + name + ' with type ' + typ)


def snapshot_profile(profile, output):
    output = Path(output)
    saved = dict(profile)
    saved['config_sha256'] = {}
    for key in ('frontend', 'std', 'geometry', 'profile_path'):
        if not profile.get(key):
            continue
        path = Path(profile[key]); data = path.read_bytes()
        saved['config_sha256'][key] = hashlib.sha256(data).hexdigest()
        (output / ('input_' + key + path.suffix)).write_bytes(data)
    (output / 'sensor_profile.json').write_text(json.dumps(saved, indent=2) + '\n')


def convert_imu(message, profile):
    """Convert component units; preserve timestamps, axes and orientation."""
    for field, covariance, scale in [('linear_acceleration', 'linear_acceleration_covariance', profile['acceleration_scale']),
                                     ('angular_velocity', 'angular_velocity_covariance', profile['angular_velocity_scale'])]:
        vec = getattr(message, field)
        vec.x *= scale; vec.y *= scale; vec.z *= scale
        values = list(getattr(message, covariance))
        # -1 denotes an unavailable measurement; all-zero unknown covariance remains zero.
        if profile['scale_covariances'] and values[0] != -1:
            setattr(message, covariance, [v * scale * scale for v in values])
    return message


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Validate sensor files without starting ROS; does not check actual bag messages')
    add_profile_arguments(parser, required=True)
    parser.add_argument('--mode', choices=['frontend', 'full'], default='frontend')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    ros = 'ros1' if (root / 'src/fast_lio').exists() else 'ros2'
    try:
        print(json.dumps(load_profile(root, ros, args.dataset, args.sensor_config, args.mode == 'full'), indent=2))
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))
