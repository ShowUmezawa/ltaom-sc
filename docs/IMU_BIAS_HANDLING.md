# IMU bias handling

The frontend follows the FAST-LIO input convention: the process model receives measured angular velocity and gravity-normalized acceleration. `get_f()` subtracts `bg` and `ba` once during state prediction. `df_dx()` retains the corresponding bias Jacobians.

After prediction, the deskew history separately uses `gyro - bg` and `R * (acc - ba) + gravity`. These corrected quantities are used for point compensation and are not fed back into the predictor.

This corrects an inherited LTA-OM path that subtracted bias before `predict()` while retaining another subtraction inside `get_f()`. Acceleration normalization is performed before subtraction of the estimator's accelerometer bias, consistent with the official FAST-LIO convention. The legacy non-IKFoM branch is unchanged.

The active deskew loop also stops after compensating the first point. Breaking only the inner loop could revisit that point in an earlier IMU segment when its timestamp lies near a segment boundary. The analytic rotation/translation case covers this boundary.

The `imu_bias_regression` C++ test exercises the actual prediction and point-deskew functions against analytic stationary and constant-yaw/translation motion, with nonzero biases, non-unit acceleration normalization and nonzero LiDAR–IMU extrinsics. It checks pose, velocity, point compensation and finite covariance. The test does not establish real-world accuracy.

Build with `BUILD_TESTING=ON` on ROS2 or `CATKIN_ENABLE_TESTING=ON` on ROS1. Execute `imu_bias_regression` in the frontend build directory (ROS2) or `devel/lib/fast_lio/imu_bias_regression` (ROS1, with a ROS master). The CTest name is `imu_bias_prediction_and_deskew`.
