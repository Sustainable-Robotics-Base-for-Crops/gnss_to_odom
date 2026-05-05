# gnss_to_odom

ROS 2 lifecycle node that fuses dual GNSS antennas and IMU to compute vehicle pose and velocity in the local ENU frame (`map`). Converts WGS84 to ENU using GeographicLib, derives yaw from antenna baseline, and publishes odometry plus optional static TF earth → map.

## Parameters

| Parameter                    | Type   | Default       | Description                                                             |
| ---------------------------- | ------ | ------------- | ----------------------------------------------------------------------- |
| `publish_tf`                 | bool   | `false`       | If true, publishes static transform from `earth` to `map` at first fix  |
| `frame_gnss_main`            | string | `"main_gnss"` | TF frame for main GNSS antenna                                          |
| `frame_gnss_aux`             | string | `"aux_gnss"`  | TF frame for auxiliary GNSS antenna                                     |
| `frame_imu`                  | string | `"imu"`       | TF frame for IMU sensor                                                 |
| `covariance_threshold`       | double | `0.01`        | Max position covariance (m²) to accept fix                              |
| `antenna_distance_tolerance` | double | `0.05`        | Tolerance (m) when comparing GNSS vs TF antenna distance for validation |

## Topics

### Publishers

| Topic            | Type                  | Description                                        |
| ---------------- | --------------------- | -------------------------------------------------- |
| `odom`           | nav_msgs/msg/Odometry | Fused odometry for `base_footprint` in `map` frame |
| `gnss/main/odom` | nav_msgs/msg/Odometry | Odometry of main GNSS antenna in `map` frame       |
| `gnss/aux/odom`  | nav_msgs/msg/Odometry | Odometry of auxiliary GNSS antenna in `map` frame  |
| `fix_vel_att`    | gps_msgs/msg/GPSFix   | Fused pose/velocity converted back to WGS84        |
| `status`         | std_msgs/msg/UInt64   | Status bitfield (see below)                        |

### Subscribers

| Topic                   | Type                | Description                        |
| ----------------------- | ------------------- | ---------------------------------- |
| `gnss/main/fix_vel_att` | gps_msgs/msg/GPSFix | Main GNSS fix/velocity             |
| `gnss/aux/fix_vel_att`  | gps_msgs/msg/GPSFix | Auxiliary GNSS fix                 |
| `imu`                   | sensor_msgs/msg/Imu | IMU orientation / angular velocity |

## Status (UInt64)

| Bit | Name                      | Description                         |
| --- | ------------------------- | ----------------------------------- |
| 0   | info_normal_operation     | Normal operation                    |
| 32  | error_no_signal_gnss_aux  | No signal from auxiliary GNSS       |
| 33  | error_no_signal_gnss_main | No signal from main GNSS            |
| 34  | error_no_signal_imu       | No IMU data                         |
| 35  | error_no_rtk_fix          | No RTK fix                          |
| 36  | error_covariance          | Position covariance above threshold |
| 37  | error_check_accuracy      | Antenna distance check failed       |

## TF

### Published (when `publish_tf` is true)

| Parent | Child          | Type    | Description                                                   |
| ------ | -------------- | ------- | ------------------------------------------------------------- |
| earth  | map            | static  | Origin of ENU frame (rounded lat/lon, rotation ENU at origin) |
| map    | base_footprint | dynamic | Vehicle pose from fused GNSS + IMU                            |

### Required static transforms

These must be provided (e.g. via URDF or `static_transform_publisher`) so the node can resolve antenna and IMU frames:

| Parent         | Child             |
| -------------- | ----------------- |
| base_footprint | `frame_gnss_main` |
| base_footprint | `frame_gnss_aux`  |
| base_footprint | `frame_imu`       |

If `publish_tf` is false, the following must also exist:

| Parent | Child |
| ------ | ----- |
| earth  | map   |
