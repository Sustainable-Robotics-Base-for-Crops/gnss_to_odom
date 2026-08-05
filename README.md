# gnss_to_odom

Lifecycle node that fuses dual GNSS antennas and IMU to publish vehicle odometry in the local ENU `map` frame.

## Overview

On each main GNSS message:

1. Convert the main (and, when available, auxiliary) GNSS fix from WGS84 to ENU.
2. Resolve yaw, in this order depending on `heading_mode`:
   - **UNIHEADING** from `GPSFix.track` when `err_track` ≤ `heading_std_max_deg`
   - **gyro hold** of the last good yaw for up to `heading_hold_timeout_s`
   - **geometric baseline** `atan2(aux − main)` (legacy / `baseline` / `auto` fallback)
3. Roll and pitch from IMU.
4. Position: mid-point of both antennas when aux is RTK-fixed; otherwise **main only**.
5. Publish `odom`, TF `map` → `base_footprint`, and fused `fix_vel_att`.

This removes the previous hard gate « both antennas must be RTK-fixed », which stopped
`/loc/odom` on every short auxiliary dropout even when the main antenna and the receiver
heading solution were still healthy.

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
| `heading_mode`               | `uniheading`     | `uniheading` \| `baseline` \| `auto`                            |
| `heading_std_max_deg`        | `2.0`            | Max `GPSFix.err_track` (deg) to accept UNIHEADING               |
| `heading_hold_timeout_s`     | `3.0`            | Max gyro-hold duration after last good yaw                      |
| `allow_main_only_position`   | `true`           | Use main antenna alone when aux is not RTK-fixed                |

## Topics

| Topic                   | Type                    | Direction | Description                                     |
| ----------------------- | ----------------------- | --------- | ----------------------------------------------- |
| `gnss/main/fix_vel_att` | `gps_msgs/msg/GPSFix`   | In        | Main GNSS fix; `track`/`err_track` = UNIHEADING |
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

| Bit  | Name                        | Description                                      |
| ---- | --------------------------- | ------------------------------------------------ |
| `0`  | `info_normal_operation`     | Reserved (info range)                            |
| `16` | `warn_heading_hold`         | Yaw is gyro-propagated (UNIHEADING temporarily down) |
| `17` | `warn_position_main_only`   | Position from main antenna only                  |
| `32` | `error_no_signal_gnss_aux`  | No auxiliary GNSS for more than 1 s              |
| `33` | `error_no_signal_gnss_main` | No main GNSS for more than 1 s                   |
| `34` | `error_no_signal_imu`       | No IMU for more than 1 s                         |
| `35` | `error_no_rtk_fix`          | Main RTK fix not available (or aux in baseline mode) |
| `36` | `error_covariance`          | Position covariance above threshold              |
| `37` | `error_check_accuracy`      | Antenna distance check failed                    |
| `38` | `error_no_rtcm`             | No RTCM for more than 5 s                        |
| `39` | `error_no_heading`          | No usable yaw (UNIHEADING, hold and baseline all failed) |

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

## Heading notes

`um982_gnss` fills `GPSFix.track` with Unicore **UNIHEADING** (degrees clockwise from True
North along the master→slave baseline) and `err_track` with the heading standard
deviation. Conversion to ENU vehicle yaw:

`yaw = normalize(from_degrees(90 − track) − yaw_offset)`

where `yaw_offset` is the angle of the antenna baseline in `base_footprint` (from URDF).
