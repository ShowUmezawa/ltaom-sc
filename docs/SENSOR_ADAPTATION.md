# Adapting your LiDAR and IMU

The custom sensor entrypoint supports **offline replay on ROS1 Noetic and ROS2 Humble**, using `sensor_msgs/PointCloud2` and `sensor_msgs/Imu`. The supported real-data configurations use Hesai 16/32-channel LiDARs. Other point cloud formats require an adapter, and Foxy support is not provided. Live sensor startup, QoS, and connection checks require separate configuration.

## 1. Create a sensor profile

Copy the entire template directory from the repository root. Relative file paths in a profile are resolved against the profile's directory, regardless of where the command is launched.

```bash
cp -r config/sensors config/my_sensor
```

Edit `config/my_sensor/custom.example.yaml`:

- `topics.lidar` / `topics.imu`: absolute topic names for the original point cloud and IMU messages in the bag.
- `imu_units.acceleration`: `m/s2` or `g`; `angular_velocity`: `rad/s` or `deg/s`. Specify both explicitly.
- `frontend`: frontend parameter YAML, using a plain nested mapping without a `ros__parameters` wrapper.
- `std`: STD configuration in OpenCV YAML; `geometry`: shared geometric verification configuration in JSON. Both are required for the full pipeline and optional for frontend-only runs.
- `retrieval`: recent-submap exclusion count `gap`, minimum time separation `seconds`, SC candidate count `top_k`, and distance threshold `sc_threshold`.

Then edit `frontend.yaml` with your channel count, time offsets, extrinsics, and noise settings. The template leaves `extrinsic_R/T` empty and refuses to start until they are filled in. Use your own calibration rather than copying the vehicle or quadruped extrinsics. Template filtering, noise, and loop settings are starting points, not calibration results for a new sensor.

Validate the configuration without starting ROS:

```bash
python3 scripts/sensor_profile.py \
  --sensor-config config/my_sensor/custom.example.yaml --mode full
```

This checks file availability, parameter structure, units, topics, rotation matrices, and time offsets. It does not inspect actual point clouds or establish calibration accuracy.

## 2. Run the pipeline

`--sensor-config` and `--dataset car|dog` are mutually exclusive. The built-in presets remain available.

### ROS1

```bash
bash scripts/replay.sh --sensor-config config/my_sensor/custom.example.yaml \
  --bag /path/to/input.bag --mode frontend --out /path/to/new_frontend
bash scripts/replay.sh --sensor-config config/my_sensor/custom.example.yaml \
  --bag /path/to/input.bag --mode dual --out /path/to/new_full
```

### ROS2

```bash
bash scripts/replay.sh frontend --sensor-config config/my_sensor/custom.example.yaml \
  --bag /path/to/rosbag2_directory --out /path/to/new_frontend
bash scripts/replay.sh full --sensor-config config/my_sensor/custom.example.yaml \
  --bag /path/to/rosbag2_directory --mode dual --out /path/to/new_full
```

The full pipeline supports `std_only`, `sc_only`, and `dual`. The output directory must not already exist. See the [project README](../README.md) for build and dependency setup. The supported ROS2 replay format is a SQLite rosbag2 directory.

## 3. Point cloud fields and timestamps

The custom entrypoint currently requires `preprocess.lidar_type: 6`, selecting this repository's Hesai parser:

| Field | PointField type | Meaning |
|---|---|---|
| `x / y / z` | FLOAT32 | Position in the LiDAR frame, in m |
| `intensity` | FLOAT32 | Intensity |
| `timestamp` | FLOAT64 | Absolute timestamp of each point, in s |
| `ring` | UINT16 | Channel index starting at 0, less than `scan_line` |

Each field must have `count=1`. Layout padding is allowed, but field offsets, `point_step`, and `row_step` must be correct. Big-endian clouds are not accepted. Clouds must contain enough valid points; setting `is_dense` does not replace validation.

- `header.stamp` must identify the scan start and differ from the earliest point timestamp by no more than `header_time_tolerance_s` (default: 0.002 s).
- Scan duration is the latest point timestamp minus the earliest. It must be positive and no greater than `max_scan_duration_s` (default: 0.2 s). Set this according to the actual scan period.
- Internally, the parser converts each point's time relative to scan start into milliseconds for motion compensation. Input timestamps must still be in absolute seconds.
- Drivers that provide relative `time`, integer nanoseconds, different field names, or an end-of-scan header timestamp need conversion in the driver or an input node, or a new validated parser. Changing `scan_line` alone does not resolve these differences.
- Do not assign a single timestamp to all points or increase tolerances to hide an incorrect timestamp convention.

## 4. IMU units and time synchronization

Acceleration must include gravity and be expressed in the selected IMU's coordinate frame. Angular velocity must use the same frame. Unit conversion does not swap axes, rotate vectors, or convert left-handed coordinates; handle these in the driver or input adapter.

For custom profiles, acceleration in `g` is multiplied by 9.80665, and angular velocity in `deg/s` by π/180. When conversion is needed, the default output topic is the original IMU topic with `_si` appended; override it with `topics.imu_output`. Input and output topics must differ to avoid a subscription loop. SI input is subscribed to directly without a relay. Conversion preserves timestamps, `frame_id`, and orientation, and scales the corresponding covariance by the square of the conversion factor. A first covariance element of -1, indicating an unavailable measurement, is preserved.

The `car` preset retains its existing behavior: scale acceleration by 9.80665 and leave message covariances unchanged. The `dog` preset uses SI input directly. Frontend noise is configured through `mapping/*_cov`; message covariances do not replace those settings.

| Frontend parameter | Operation performed by the code |
|---|---|
| `common.time_offset_lidar_to_imu` | `corrected_IMU_time = raw_IMU_time − offset` |
| `common.time_offset_from_lidar` | `corrected_LiDAR_scan_time = raw_LiDAR_scan_time + offset` |

Both offsets are in seconds and must not both be nonzero. For example, if the IMU timestamp for the same event is 0.02 s later than the LiDAR timestamp, set the first parameter to `0.02`. Both default to zero. The vehicle preset's `-36.89` is specific to its data, not a general Hesai setting. A constant offset cannot correct clock drift, out-of-order messages, or variable host reception latency. Point/header timestamp consistency is checked before this overall clock offset is applied.

## 5. Extrinsic direction

`mapping.extrinsic_R` contains nine rotation matrix elements in row-major order. `mapping.extrinsic_T` contains three translation elements in m. The convention is **LiDAR → IMU**:

```text
p_IMU = extrinsic_R × p_LiDAR + extrinsic_T
```

The translation is the LiDAR origin expressed in the IMU frame. If calibration provides the opposite direction, use `R_inverse = Rᵀ` and `t_inverse = −Rᵀt`. The rotation must be orthonormal with determinant +1. Identity rotation and zero translation apply only when they match the physical installation. Extrinsics are not inferred automatically from message `frame_id` values or TF.

`mapping.extrinsic_est_en: false` uses fixed extrinsics. Setting it to true enables online estimation, which still requires a reasonable initial estimate and sufficient motion excitation. First validate the frontend with an independent calibration before considering online estimation. When using a robot body IMU, use the LiDAR-to-body-IMU calibration, not the calibration for a different IMU.

## 6. Main frontend parameters

| Parameter | Purpose and tuning basis |
|---|---|
| `preprocess.scan_line` | Must cover the driver's actual ring indices; the parser accepts 1–128 channels, with real-data configurations for 16/32 channels |
| `preprocess.blind` | Near-range exclusion radius in m, for occlusions and blind zones |
| `point_filter_num` | Point sequence sampling stride, ≥1; larger values retain fewer points |
| `feature_extract_enable` | Must be false for the current Hesai adapter |
| `filter_size_surf` / `filter_size_map` | Scan/map voxel sizes in m; tune for point density, scene scale, and compute budget |
| `mapping.det_range` / `mapping.fov_degree` | Range and field-of-view settings used in map maintenance, in m/degrees; set for effective sensor range and field of view, without relying on them to remove all invalid input points |
| `cube_side_length` | Local map cube side length in m |
| `max_iteration` | Maximum number of filter update iterations |
| `mapping.acc_cov` / `mapping.gyr_cov` | Acceleration/angular velocity process noise covariance settings; the same value is used on each axis |
| `mapping.b_acc_cov` / `mapping.b_gyr_cov` | Accelerometer/gyroscope bias random-walk process noise covariance settings |

These noise values populate the filter's process noise matrix. Convert and validate calibration results according to measurement units, sample rate, and the filter's discretization convention; do not copy a datasheet noise density or standard deviation directly into these fields. The supplied vehicle and robot settings are not universal defaults.

## 7. Loop retrieval and geometric verification

Check frontend continuity, motion compensation, and extrinsics before enabling loop closure. Main settings include:

- **STD YAML:** `sub_frame_num` controls scan accumulation per submap, subject to motion thresholds; stationary scans can continue accumulating into the final submap. `ds_size`, `voxel_size`, and `proj_image_resolution` set spatial scales. `descriptor_min_len/max_len` control triangle edge lengths; `skip_near_num` excludes nearby submaps.
- **Profile `retrieval`:** shared submap/time exclusion rules, SC candidate count, and matching threshold. The shared layer cannot restore candidates already excluded by STD.
- **Scan Context descriptor:** defaults are 20×60, an 80 m radius, and a 2 m height encoding offset. These are C++ descriptor settings, not fields changed by this sensor YAML. The 2 m value is not a mounting-height extrinsic.
- **Geometry JSON:** `voxel_m`, `min_points`, `max_correspondence_m`, `min_overlap`, residual thresholds, spatial coverage, and degeneracy thresholds. `rotation_sigma_rad` and `translation_sigma_m` are loop factor noise standard deviations, ordered rotation then translation.
- **`allow_assumed_gravity`:** controls whether an independently unvalidated frontend gravity assumption may be used. The template references a configuration with this set to false, so SC may produce no verified loops. Do not enable it solely to increase the loop count.

Suitable values depend on the scene. Reducing scans per submap also changes the actual time span covered by `gap` and STD's nearby-submap exclusion count; consider these settings together.

## 8. Override order and effective parameters

The frontend file referenced by the sensor profile is loaded first. ROS2's `--params-override` JSON is then merged recursively by nested field, and the entrypoint applies its runtime settings last. The JSON can override filtering, noise, and other non-reserved parameters. It cannot change topics, output directories, tree rebuild mode, frontend feedback, multisession mode, or end-of-input counts; conflicting overrides raise an error. Overrides use recursive merging rather than replacing an entire nested section.

| Setting | Final source of control |
|---|---|
| `common.lid_topic/imu_topic` | Sensor profile, overriding the frontend YAML; IMU uses the converted output topic or the original SI topic |
| `SaveDir` | `--out`; ROS1 uses the output root, while the ROS2 frontend uses its `frontend/` subdirectory |
| `frontend_feedback_en` / `multisession_mode` | Fixed to false / 0 by the replay entrypoint |
| `ikdtree_rebuild_mode` | Fixed to sync on ROS1; selected by `--rebuild-mode` on ROS2, defaulting to sync |
| Frontend timing, extrinsics, noise, and cloud filtering | Frontend YAML; ROS2 may override non-reserved fields through the JSON above |
| STD `lcd_config_path` | Profile's `std` file, which controls submap scale and other STD settings |
| STD candidate export/retrieval | Selected by `--mode`; the entrypoint enables candidate mode and final-submap flushing |
| Retrieval separation and SC thresholds | Profile's `retrieval`; defaults are 50 submaps / 30 s / top-10 / 0.13 |
| Geometric verification and loop noise | Profile's `geometry` file |
| Backend and end-of-input handling | Entrypoint sets session, archive paths, runtime mode, and expected message counts; these are not sensor-profile fields |
| ROS2 transport and replay | Reliable/volatile input QoS; LiDAR depth 256 and IMU depth 8192; 1× replay with automatic flow control |

ROS1 also loads the repository's backend defaults, then sets `backend_mode=rebuildable` and `loop_input_mode=verified_pose3`, disables adjacent corner constraints and factor compression, and enables map saving. ROS2 uses the separate backend node's default thresholds; the entrypoint sets its output directory, session, and map sampling stride. A sensor profile is not a general interface for passing arbitrary backend parameters.

Each run saves `sensor_profile.json`, copies of input configuration files, and SHA256 hashes in the output directory. ROS1 saves `frontend_params.yaml` and the final ROS parameters in `parameters.yaml`. ROS2 saves `params.yaml`, the full pipeline's `std_params.yaml/backend_params.yaml`, and input `qos.yaml`. Use these files to inspect the configuration actually used.

## 9. Suggested adaptation workflow

1. Check message types, point fields, ring indices, scan time spans, IMU frequency, and timestamp continuity.
2. Confirm IMU units and axes, and enter calibrated extrinsics and time offsets. Provide a stationary segment at the start for initialization.
3. Replay a short segment with the frontend only. Check `invalid_lidar`, input counts, trajectory continuity, and duplicated map surfaces.
4. Once the frontend behaves correctly, tune submap scale and run the full pipeline. Inspect candidate rejection reasons and map geometry.
5. Compare trajectory errors when ground truth is available. Without ground truth, more loops or a visually smoother map do not by themselves establish better localization accuracy.
