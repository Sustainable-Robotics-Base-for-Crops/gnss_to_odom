# gnss_to_odom

Lifecycle node that fuses dual GNSS antennas and IMU to publish vehicle odometry in the local ENU `map` frame.

## Overview

On each main GNSS message (when auxiliary data is fresh within 0.2 s):

1. Convert both GNSS fixes from WGS84 to ENU (GeographicLib).
2. Fuse yaw from the antenna baseline, roll and pitch from IMU.
3. Compute `base_footprint` pose and velocity.
4. Validate RTK fix, covariance and antenna distance consistency.
5. Publish `odom`, per-antenna odometry, fused WGS84 `fix_vel_att`, and `map` → `base_footprint` TF.

Per-antenna odometry (`gnss/main/odom`, `gnss/aux/odom`) is published on every valid fix.

Fused `odom` is published only when both antennas have RTK fix, horizontal covariance is below `covariance_threshold`, and the antenna distance check passes.

At activate, static transforms from `base_footprint` to the GNSS and IMU frames are looked up. Activation fails if any lookup fails.

On first fix, the ENU origin is initialized. If `publish_tf` is true, a static `earth` → `map` transform is published. Otherwise `earth` → `map` is read from TF.

A 1 Hz timer publishes `status`. IMU data older than 0.2 s is ignored (roll and pitch set to zero) unless `disable_imu` is true. The status bit `error_no_rtcm` is set when no RTCM message arrived for more than 5 s.

## Parameters

| Parameter                    | Default (header) | Description                                                     |
| ---------------------------- | ---------------- | --------------------------------------------------------------- |
| `publish_tf`                 | `false`          | Publish static `earth` → `map` at first fix                     |
| `disable_imu`                | `false`          | Ignore IMU input (roll, pitch and angular velocity)             |
| `frame_gnss_main`            | `main_gnss`      | TF frame for main GNSS antenna                                  |
| `frame_gnss_aux`             | `aux_gnss`       | TF frame for auxiliary GNSS antenna                             |
| `frame_imu`                  | `imu`            | TF frame for IMU                                                |
| `covariance_threshold`       | `0.01`           | Max horizontal position variance (m²) to publish fused odometry |
| `antenna_distance_tolerance` | `0.05`           | Max error (m) on antenna baseline distance check                |

## Topics

| Topic                   | Type                    | Direction | Description                                     |
| ----------------------- | ----------------------- | --------- | ----------------------------------------------- |
| `gnss/main/fix_vel_att` | `gps_msgs/msg/GPSFix`   | In        | Main GNSS fix and velocity                      |
| `gnss/aux/fix_vel_att`  | `gps_msgs/msg/GPSFix`   | In        | Auxiliary GNSS fix                              |
| `gnss/rtcm`             | `mavros_msgs/msg/RTCM`  | In        | RTCM corrections (watchdog only)                |
| `imu`                   | `sensor_msgs/msg/Imu`   | In        | IMU orientation and angular velocity            |
| `odom`                  | `nav_msgs/msg/Odometry` | Out       | Fused odometry for `base_footprint` in `map`    |
| `gnss/main/odom`        | `nav_msgs/msg/Odometry` | Out       | Main antenna odometry in `map`                  |
| `gnss/aux/odom`         | `nav_msgs/msg/Odometry` | Out       | Auxiliary antenna odometry in `map`             |
| `fix_vel_att`           | `gps_msgs/msg/GPSFix`   | Out       | Fused pose and velocity converted back to WGS84 |
| `status`                | `std_msgs/msg/UInt64`   | Out       | Localization status bitmask (1 Hz)              |

Subscribers are created in the active lifecycle state.

`status` bitmask:

| Bit  | Name                        | Description                         |
| ---- | --------------------------- | ----------------------------------- |
| `0`  | `info_normal_operation`     | Reserved (info range)               |
| `32` | `error_no_signal_gnss_aux`  | No auxiliary GNSS for more than 1 s |
| `33` | `error_no_signal_gnss_main` | No main GNSS for more than 1 s      |
| `34` | `error_no_signal_imu`       | No IMU for more than 1 s            |
| `35` | `error_no_rtk_fix`          | RTK fix not available               |
| `36` | `error_covariance`          | Position covariance above threshold |
| `37` | `error_check_accuracy`      | Antenna distance check failed       |
| `38` | `error_no_rtcm`             | No RTCM for more than 5 s           |

Required static transforms (URDF or `static_transform_publisher`):

| Parent           | Child             |
| ---------------- | ----------------- |
| `base_footprint` | `frame_gnss_main` |
| `base_footprint` | `frame_gnss_aux`  |
| `base_footprint` | `frame_imu`       |

Published transforms:

| Parent  | Child            | Type    | Condition                       |
| ------- | ---------------- | ------- | ------------------------------- |
| `earth` | `map`            | static  | `publish_tf` true, at first fix |
| `map`   | `base_footprint` | dynamic | On each published fused `odom`  |

If `publish_tf` is false, `earth` → `map` must already exist in TF.
