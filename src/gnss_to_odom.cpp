// Copyright 2026 SABI AGRI

#include "gnss_to_odom/gnss_to_odom.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "angles/angles.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace gnss_to_odom
{
GnssToOdom::GnssToOdom(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("gnss_to_odom", options)
{
  this->declare_parameter("publish_tf", publish_tf_);
  this->declare_parameter("disable_imu", disable_imu_);
  this->declare_parameter("frame_gnss_main", frame_gnss_main_);
  this->declare_parameter("frame_gnss_aux", frame_gnss_aux_);
  this->declare_parameter("frame_imu", frame_imu_);
  this->declare_parameter("covariance_threshold", covariance_threshold_);
  this->declare_parameter("antenna_distance_tolerance", antenna_distance_tolerance_);
}

LNI::CallbackReturn GnssToOdom::on_configure(const rclcpp_lifecycle::State&)
{
  this->get_parameter("publish_tf", publish_tf_);
  this->get_parameter("disable_imu", disable_imu_);
  this->get_parameter("frame_gnss_main", frame_gnss_main_);
  this->get_parameter("frame_gnss_aux", frame_gnss_aux_);
  this->get_parameter("frame_imu", frame_imu_);
  this->get_parameter("covariance_threshold", covariance_threshold_);
  this->get_parameter("antenna_distance_tolerance", antenna_distance_tolerance_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  tf_odom_.header.frame_id = "map";
  tf_odom_.child_frame_id = "base_footprint";

  msg_odom_.header.frame_id = "map";
  msg_odom_.child_frame_id = "base_footprint";
  msg_odom_main_.header.frame_id = "map";
  msg_odom_main_.child_frame_id = frame_gnss_main_;
  msg_odom_aux_.header.frame_id = "map";
  msg_odom_aux_.child_frame_id = frame_gnss_aux_;

  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  pub_odom_main_ = this->create_publisher<nav_msgs::msg::Odometry>("gnss/main/odom", 10);
  pub_odom_aux_ = this->create_publisher<nav_msgs::msg::Odometry>("gnss/aux/odom", 10);
  pub_odom_wgs84_ = this->create_publisher<gps_msgs::msg::GPSFix>("fix_vel_att", 10);
  pub_status_ = this->create_publisher<std_msgs::msg::UInt64>("status", 10);

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn GnssToOdom::on_activate(const rclcpp_lifecycle::State& state)
{
  LifecycleNode::on_activate(state);

  std::string id = this->get_namespace();

  if (id != "/")
  {
    id += "/";
  }

  id += this->get_name();

  bond_ = std::make_unique<bond::Bond>("bond", id, shared_from_this());
  bond_->start();

  // Look up for the transformation between base_footprint and imu frame
  geometry_msgs::msg::TransformStamped tf_imu;

  try
  {
    tf_imu = tf_buffer_->lookupTransform("base_footprint", frame_imu_, tf2::TimePointZero, tf2::durationFromSec(60));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not transform base_footprint to %s: %s", frame_imu_.c_str(), ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  tf2::fromMsg(tf_imu.transform.rotation, orientation_tf_imu_);

  // Look up for the transformation between base_footprint and gnss main frame
  geometry_msgs::msg::TransformStamped tf_gnss_main;

  try
  {
    tf_gnss_main =
        tf_buffer_->lookupTransform("base_footprint", frame_gnss_main_, tf2::TimePointZero, tf2::durationFromSec(60));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not transform base_footprint to %s: %s", frame_gnss_main_.c_str(),
                 ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  tf2::fromMsg(tf_gnss_main.transform.translation, position_tf_gnss_main_);

  RCLCPP_INFO_STREAM(this->get_logger(), "GNSS main position x: " << position_tf_gnss_main_.x()
                                                                  << ", y: " << position_tf_gnss_main_.y()
                                                                  << ", z: " << position_tf_gnss_main_.z());

  // Look up for the transformation between base_footprint and gnss aux frame
  geometry_msgs::msg::TransformStamped tf_gnss_aux;

  try
  {
    tf_gnss_aux =
        tf_buffer_->lookupTransform("base_footprint", frame_gnss_aux_, tf2::TimePointZero, tf2::durationFromSec(60));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not transform base_footprint to %s: %s", frame_gnss_aux_.c_str(),
                 ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  tf2::fromMsg(tf_gnss_aux.transform.translation, position_tf_gnss_aux_);

  RCLCPP_INFO_STREAM(this->get_logger(), "GNSS aux position x: " << position_tf_gnss_aux_.x()
                                                                 << ", y: " << position_tf_gnss_aux_.y()
                                                                 << ", z: " << position_tf_gnss_aux_.z());

  // Central point between GNSS antennas
  position_tf_gnss_.setX((position_tf_gnss_main_.getX() + position_tf_gnss_aux_.getX()) / 2.);
  position_tf_gnss_.setY((position_tf_gnss_main_.getY() + position_tf_gnss_aux_.getY()) / 2.);
  position_tf_gnss_.setZ((position_tf_gnss_main_.getZ() + position_tf_gnss_aux_.getZ()) / 2.);

  RCLCPP_INFO_STREAM(this->get_logger(), "GNSS central point position x: " << position_tf_gnss_.x()
                                                                           << ", y: " << position_tf_gnss_.y()
                                                                           << ", z: " << position_tf_gnss_.z());

  // Yaw offset from GNSS antennas position
  yaw_offset_ = atan2(tf_gnss_aux.transform.translation.y - tf_gnss_main.transform.translation.y,
                      tf_gnss_aux.transform.translation.x - tf_gnss_main.transform.translation.x);

  RCLCPP_INFO_STREAM(this->get_logger(), "Yaw offset: " << yaw_offset_);

  time_imu_ = this->now();
  time_rtcm_ = this->now();
  time_gnss_main_ = this->now();
  time_gnss_aux_ = this->now();

  sub_gnss_main_ = this->create_subscription<gps_msgs::msg::GPSFix>(
      "gnss/main/fix_vel_att", 10, std::bind(&GnssToOdom::gnss_main_callback, this, _1));

  sub_gnss_aux_ = this->create_subscription<gps_msgs::msg::GPSFix>("gnss/aux/fix_vel_att", 10,
                                                                   std::bind(&GnssToOdom::gnss_aux_callback, this, _1));

  sub_rtcm_ = this->create_subscription<mavros_msgs::msg::RTCM>("gnss/rtcm", 10,
                                                                std::bind(&GnssToOdom::rtcm_callback, this, _1));

  sub_imu_ =
      this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&GnssToOdom::imu_callback, this, _1));

  timer_status_ = this->create_wall_timer(1s, std::bind(&GnssToOdom::status_timer_callback, this));

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn GnssToOdom::on_deactivate(const rclcpp_lifecycle::State& state)
{
  bond_->breakBond();
  LifecycleNode::on_deactivate(state);
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn GnssToOdom::on_cleanup(const rclcpp_lifecycle::State&)
{
  bond_.reset();

  pub_odom_.reset();
  pub_odom_main_.reset();
  pub_odom_aux_.reset();
  pub_odom_wgs84_.reset();
  pub_status_.reset();

  timer_status_.reset();

  sub_gnss_main_.reset();
  sub_gnss_aux_.reset();
  sub_rtcm_.reset();
  sub_imu_.reset();

  tf_buffer_.reset();
  tf_listener_.reset();
  tf_broadcaster_.reset();
  tf_static_broadcaster_.reset();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn GnssToOdom::on_shutdown(const rclcpp_lifecycle::State&)
{
  bond_.reset();

  pub_odom_.reset();
  pub_odom_main_.reset();
  pub_odom_aux_.reset();
  pub_odom_wgs84_.reset();
  pub_status_.reset();

  timer_status_.reset();

  sub_gnss_main_.reset();
  sub_gnss_aux_.reset();
  sub_rtcm_.reset();
  sub_imu_.reset();

  tf_buffer_.reset();
  tf_listener_.reset();
  tf_broadcaster_.reset();
  tf_static_broadcaster_.reset();

  return LNI::CallbackReturn::SUCCESS;
}

/// \brief Converts a GPSFix message to an Odometry message
/// \param[in] fix GPSFix message containing GNSS position, orientation, and velocity data
/// \param[out] odom Odometry message to be populated with converted data
///
/// This function converts GNSS data from WGS84 coordinates to ENU (East-North-Up) local coordinates,
/// extracts the track angle as orientation, and copies position covariance and speed information.
void GnssToOdom::fix_to_odom(const gps_msgs::msg::GPSFix::SharedPtr& fix, nav_msgs::msg::Odometry& odom)
{
  odom.header.stamp = fix->header.stamp;

  wgs84_to_enu(fix->latitude, fix->longitude, fix->altitude, odom.pose.pose.position);

  tf2::Quaternion q;
  q.setRPY(0, 0, fix->track / 180. * M_PI);
  odom.pose.pose.orientation = tf2::toMsg(q);

  // Copy position covariance from GPSFix to Odometry
  // GPSFix uses a 3x3 covariance matrix (9 elements) stored row-major: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
  // Odometry uses a 6x6 covariance matrix (36 elements) stored row-major for [x, y, z, roll, pitch, yaw]
  // We only copy the diagonal elements (variances) for x, y, z positions
  odom.pose.covariance[0] = fix->position_covariance[0];   // Variance in x (index 0 in 6x6 matrix)
  odom.pose.covariance[7] = fix->position_covariance[4];   // Variance in y (index 7 = 1*6+1 in 6x6 matrix)
  odom.pose.covariance[14] = fix->position_covariance[8];  // Variance in z (index 14 = 2*6+2 in 6x6 matrix)

  odom.twist.twist.linear.x = fix->speed;
}

/// \brief Converts WGS84 coordinates (latitude, longitude, altitude) to ENU (East-North-Up) local coordinates
/// \param[in] latitude WGS84 latitude in degrees
/// \param[in] longitude WGS84 longitude in degrees
/// \param[in] altitude WGS84 altitude in meters
/// \param[out] position Point in ENU local coordinates (map frame)
///
/// This function initializes the ENU coordinate system on first call by either:
/// - Publishing a static transform from "earth" to "map" frame if publish_tf_ is true
/// - Looking up the existing transform from TF tree if publish_tf_ is false
/// The origin is set using rounded coordinates (0.01 degree precision) and adjusted altitude
/// (GNSS antenna altitude minus the central point height relative to base_footprint).
/// Subsequent calls convert the provided WGS84 coordinates to ENU using the initialized system.
void GnssToOdom::wgs84_to_enu(double latitude, double longitude, double altitude, geometry_msgs::msg::Point& position)
{
  if (std::isnan(latitude) || std::isnan(longitude) || std::isnan(altitude))
  {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Cannot use wgs84 position : lat " << latitude << " ; lon " << longitude << " ; alt "
                                                                   << altitude << " ; NaN values");
    return;
  }

  if (!enu_initialized_)
  {
    if (publish_tf_)
    {
      RCLCPP_INFO(this->get_logger(), "Publish static tf from earth to map");

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->now();
      t.header.frame_id = "earth";
      t.child_frame_id = "map";

      const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();

      double latitude_rounded = round(latitude * 100) / 100.;
      double longitude_rounded = round(longitude * 100) / 100.;
      double altitude_map = altitude - position_tf_gnss_.z();

      earth.Forward(latitude_rounded, longitude_rounded, altitude_map, t.transform.translation.x,
                    t.transform.translation.y, t.transform.translation.z);

      // Rotation ENU -> ECEF so that map frame axes are East-North-Up at origin
      const double lat_rad = latitude_rounded * M_PI / 180.;
      const double lon_rad = longitude_rounded * M_PI / 180.;
      const double clat = std::cos(lat_rad), slat = std::sin(lat_rad);
      const double clon = std::cos(lon_rad), slon = std::sin(lon_rad);
      tf2::Matrix3x3 R_enu_to_ecef(-slon, -slat * clon, clat * clon, clon, -slat * slon, clat * slon, 0., clat, slat);
      tf2::Quaternion q_enu_to_ecef;
      R_enu_to_ecef.getRotation(q_enu_to_ecef);
      t.transform.rotation = tf2::toMsg(q_enu_to_ecef);

      tf_static_broadcaster_->sendTransform(t);

      enu_.Reset(latitude_rounded, longitude_rounded, altitude_map);
    }
    else
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Look up for the transformation between earth and map");

      geometry_msgs::msg::TransformStamped t;

      try
      {
        t = tf_buffer_->lookupTransform("earth", "map", tf2::TimePointZero, tf2::durationFromSec(60));
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_ERROR(this->get_logger(), "Could not transform earth to map: %s", ex.what());
        return;
      }

      double lat, lon, alt;

      const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();

      earth.Reverse(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, lat, lon, alt);
      enu_.Reset(lat, lon, alt);
    }

    enu_initialized_ = true;
  }

  enu_.Forward(latitude, longitude, altitude, position.x, position.y, position.z);
}

/// \brief Updates the odometry message by fusing data from dual GNSS antennas and IMU
///
/// This function computes the vehicle's pose and velocity by combining:
/// - Dual GNSS antenna positions to determine yaw angle and average position
/// - IMU data for roll and pitch angles
/// - Transformations to compute base_footprint position from GNSS antenna positions
///
/// The process includes:
/// 1. Yaw calculation from the relative position of the two GNSS antennas
/// 2. Orientation fusion: yaw from GNSS combined with roll/pitch from IMU
/// 3. Position computation: average of both GNSS antenna positions, then transformed
///    to base_footprint frame accounting for the GNSS antenna offset
/// 4. Covariance propagation: maximum covariance values from both antennas
/// 5. Velocity estimation: linear speed from GNSS data with sign determined by motion direction
///
/// If IMU data is stale (>0.2s), roll and pitch are set to zero and angular velocity is cleared.
void GnssToOdom::update_odometry()
{
  msg_odom_.header.stamp = this->now();

  if ((this->now() - time_imu_).seconds() > 0.2 || disable_imu_)
  {
    if (!disable_imu_)
    {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "No IMU data for : " << (this->now() - time_imu_).seconds() << "s");
    }

    orientation_imu_.x = 0;
    orientation_imu_.y = 0;
    angular_velocity_imu_.z = 0;
  }

  // Orientation
  double yaw = atan2(msg_odom_aux_.pose.pose.position.y - msg_odom_main_.pose.pose.position.y,
                     msg_odom_aux_.pose.pose.position.x - msg_odom_main_.pose.pose.position.x);

  yaw = angles::normalize_angle(yaw - yaw_offset_);

  tf2::Quaternion q;
  q.setRPY(orientation_imu_.x, orientation_imu_.y, yaw);
  msg_odom_.pose.pose.orientation = tf2::toMsg(q);

  // Position
  geometry_msgs::msg::Pose pose;
  pose.position.x = (msg_odom_main_.pose.pose.position.x + msg_odom_aux_.pose.pose.position.x) / 2.;
  pose.position.y = (msg_odom_main_.pose.pose.position.y + msg_odom_aux_.pose.pose.position.y) / 2.;
  pose.position.z = (msg_odom_main_.pose.pose.position.z + msg_odom_aux_.pose.pose.position.z) / 2.;

  tf2::Quaternion odom_orientation;
  tf2::fromMsg(msg_odom_.pose.pose.orientation, odom_orientation);

  tf2::Transform gnss_base_tf;
  gnss_base_tf.setOrigin(tf2::quatRotate(odom_orientation, position_tf_gnss_));
  gnss_base_tf.setRotation(tf2::Quaternion::getIdentity());

  tf2::Transform gnss_map_tf;
  tf2::fromMsg(pose, gnss_map_tf);

  tf2::Transform base_map_tf = gnss_base_tf.inverse() * gnss_map_tf;

  geometry_msgs::msg::Point position_last = msg_odom_.pose.pose.position;
  tf2::toMsg(base_map_tf.getOrigin(), msg_odom_.pose.pose.position);

  msg_odom_.pose.covariance[0] = std::max(msg_odom_main_.pose.covariance[0], msg_odom_aux_.pose.covariance[0]);
  msg_odom_.pose.covariance[7] = std::max(msg_odom_main_.pose.covariance[7], msg_odom_aux_.pose.covariance[7]);
  msg_odom_.pose.covariance[14] = std::max(msg_odom_main_.pose.covariance[14], msg_odom_aux_.pose.covariance[14]);

  // Speed
  double dx = msg_odom_.pose.pose.position.x - position_last.x;
  double dy = msg_odom_.pose.pose.position.y - position_last.y;
  double s = dx * cos(yaw) + dy * sin(yaw);

  if (s > 1e-4)
  {
    odom_speed_sign_ = 1;
  }
  else if (s < 1e-4)
  {
    odom_speed_sign_ = -1;
  }

  msg_odom_.twist.twist.linear.x =
      odom_speed_sign_ * (msg_odom_main_.twist.twist.linear.x + msg_odom_aux_.twist.twist.linear.x) / 2.;
  msg_odom_.twist.twist.angular.z = angular_velocity_imu_.z;
}

/// \brief Validates the accuracy of GNSS measurements by comparing antenna distances
/// \return true if the distance difference is within tolerance, false otherwise
///
/// This function performs a consistency check between two methods of computing the distance
/// between the dual GNSS antennas:
/// 1. Distance from GNSS topic data: computed from the positions of both antennas in the map frame
/// 2. Distance from static transforms: computed from the known antenna positions in base_footprint
///    frame, transformed to map frame using the current vehicle orientation
///
/// The comparison is done in 2D (xy plane only) to avoid altitude-related inaccuracies.
/// If the absolute difference between the two distances exceeds antenna_distance_tolerance_,
/// a warning is logged and the function returns false, indicating a potential accuracy issue.
bool GnssToOdom::check_accuracy()
{
  // GNSS antennas 2D distance from topics
  tf2::Vector3 distance_gnss;
  distance_gnss.setX(msg_odom_main_.pose.pose.position.x - msg_odom_aux_.pose.pose.position.x);
  distance_gnss.setY(msg_odom_main_.pose.pose.position.y - msg_odom_aux_.pose.pose.position.y);
  distance_gnss.setZ(0.);

  // GNSS antennas 2D distance from tf
  tf2::Quaternion odom_orientation;
  tf2::fromMsg(msg_odom_.pose.pose.orientation, odom_orientation);

  tf2::Vector3 position_main_map = tf2::quatRotate(odom_orientation, position_tf_gnss_main_);
  tf2::Vector3 position_aux_map = tf2::quatRotate(odom_orientation, position_tf_gnss_aux_);

  tf2::Vector3 distance_tf;
  distance_tf.setX(position_main_map.x() - position_aux_map.x());
  distance_tf.setY(position_main_map.y() - position_aux_map.y());
  distance_tf.setZ(0.);

  if (fabs(distance_tf.length() - distance_gnss.length()) > antenna_distance_tolerance_)
  {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Antennas distance measured from tf: " << distance_tf.length()
                                                                       << "; Antennas distance measured from GNSS: "
                                                                       << distance_gnss.length());
    return false;
  }

  return true;
}

void GnssToOdom::publish()
{
  if (gnss_main_status_ < gps_msgs::msg::GPSStatus::STATUS_GBAS_FIX)
  {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Cannot publish odom msg : GNSS main status "
                                    << gnss_main_status_ << " (must be >= " << gps_msgs::msg::GPSStatus::STATUS_GBAS_FIX
                                    << ")");

    msg_status_.data |= (1ULL << uint8_t(LocStatus::error_no_rtk_fix));

    return;
  }

  msg_status_.data &= ~(1ULL << uint8_t(LocStatus::error_no_rtk_fix));

  if (gnss_aux_status_ < gps_msgs::msg::GPSStatus::STATUS_GBAS_FIX)
  {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Cannot publish odom msg : GNSS aux status "
                                    << gnss_aux_status_ << " (must be >= " << gps_msgs::msg::GPSStatus::STATUS_GBAS_FIX
                                    << ")");

    msg_status_.data |= (1ULL << uint8_t(LocStatus::error_no_rtk_fix));

    return;
  }

  msg_status_.data &= ~(1ULL << uint8_t(LocStatus::error_no_rtk_fix));

  // covariance[0] = variance in x, covariance[7] = variance in y
  double cov_main_horz = std::max(msg_odom_main_.pose.covariance[0], msg_odom_main_.pose.covariance[7]);
  double cov_aux_horz = std::max(msg_odom_aux_.pose.covariance[0], msg_odom_aux_.pose.covariance[7]);

  if (cov_main_horz > covariance_threshold_)
  {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Cannot publish odom msg : GNSS main covariance too high ("
                                    << cov_main_horz << " > " << covariance_threshold_ << ")");

    msg_status_.data |= (1ULL << uint8_t(LocStatus::error_covariance));

    return;
  }

  msg_status_.data &= ~(1ULL << uint8_t(LocStatus::error_covariance));

  if (cov_aux_horz > covariance_threshold_)
  {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Cannot publish odom msg : GNSS aux covariance too high ("
                                    << cov_aux_horz << " > " << covariance_threshold_ << ")");

    msg_status_.data |= (1ULL << uint8_t(LocStatus::error_covariance));

    return;
  }

  msg_status_.data &= ~(1ULL << uint8_t(LocStatus::error_covariance));

  if (!check_accuracy())
  {
    msg_status_.data |= (1ULL << uint8_t(LocStatus::error_check_accuracy));
    return;
  }

  msg_status_.data &= ~(1ULL << uint8_t(LocStatus::error_check_accuracy));

  pub_odom_->publish(msg_odom_);

  // Publish WGS84 odometry message
  gps_msgs::msg::GPSFix msg;

  msg.header = msg_odom_.header;

  msg.status.status = gps_msgs::msg::GPSStatus::STATUS_GBAS_FIX;
  msg.status.motion_source = gps_msgs::msg::GPSStatus::SOURCE_POINTS;
  msg.status.orientation_source = gps_msgs::msg::GPSStatus::SOURCE_POINTS + gps_msgs::msg::GPSStatus::SOURCE_ACCEL +
                                  gps_msgs::msg::GPSStatus::SOURCE_GYRO;
  msg.status.position_source = gps_msgs::msg::GPSStatus::SOURCE_GPS;

  enu_.Reverse(msg_odom_.pose.pose.position.x, msg_odom_.pose.pose.position.y, msg_odom_.pose.pose.position.z,
               msg.latitude, msg.longitude, msg.altitude);

  msg.speed = msg_odom_.twist.twist.linear.x;

  tf2::Quaternion q;
  tf2::fromMsg(msg_odom_.pose.pose.orientation, q);
  tf2::Matrix3x3 m(q);
  m.getRPY(msg.roll, msg.pitch, msg.dip);
  msg.track = msg.dip;

  msg.position_covariance[0] = msg_odom_.pose.covariance[0];
  msg.position_covariance[4] = msg_odom_.pose.covariance[7];
  msg.position_covariance[8] = msg_odom_.pose.covariance[14];

  msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  pub_odom_wgs84_->publish(msg);

  // Send the transformation between base_footprint and map
  tf_odom_.header.stamp = msg_odom_.header.stamp;
  tf_odom_.transform.translation.x = msg_odom_.pose.pose.position.x;
  tf_odom_.transform.translation.y = msg_odom_.pose.pose.position.y;
  tf_odom_.transform.translation.z = msg_odom_.pose.pose.position.z;
  tf_odom_.transform.rotation = msg_odom_.pose.pose.orientation;
  tf_broadcaster_->sendTransform(tf_odom_);
}

void GnssToOdom::gnss_main_callback(const gps_msgs::msg::GPSFix::SharedPtr msg)
{
  if (msg->status.status == gps_msgs::msg::GPSStatus::STATUS_NO_FIX)
  {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Cannot use gps_fix msg from GNSS main : status " << msg->status.status);
    return;
  }

  gnss_main_status_ = msg->status.status;

  fix_to_odom(msg, msg_odom_main_);
  pub_odom_main_->publish(msg_odom_main_);

  time_gnss_main_ = this->now();

  if ((this->now() - time_gnss_aux_).seconds() > 0.2)
  {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "No GNSS aux data for : " << (this->now() - time_gnss_aux_).seconds() << "s");
    return;
  }

  update_odometry();
  publish();
}

void GnssToOdom::gnss_aux_callback(const gps_msgs::msg::GPSFix::SharedPtr msg)
{
  if (msg->status.status == gps_msgs::msg::GPSStatus::STATUS_NO_FIX)
  {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Cannot use gps_fix msg from GNSS aux : status " << msg->status.status);
    return;
  }

  gnss_aux_status_ = msg->status.status;

  fix_to_odom(msg, msg_odom_aux_);
  pub_odom_aux_->publish(msg_odom_aux_);

  time_gnss_aux_ = this->now();
}

void GnssToOdom::rtcm_callback(const mavros_msgs::msg::RTCM::SharedPtr)
{
  time_rtcm_ = this->now();
}

void GnssToOdom::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (disable_imu_)
  {
    return;
  }

  // base_footprint orientation (roll, pitch, yaw)
  tf2::Quaternion q;
  tf2::fromMsg(msg->orientation, q);
  q = q * orientation_tf_imu_.inverse();
  tf2::Matrix3x3 m(q);
  m.getRPY(orientation_imu_.x, orientation_imu_.y, orientation_imu_.z);

  // base_footprint angular velocity
  tf2::Vector3 angular_velocity;
  angular_velocity.setX(msg->angular_velocity.x);
  angular_velocity.setY(msg->angular_velocity.y);
  angular_velocity.setZ(msg->angular_velocity.z);
  angular_velocity = tf2::quatRotate(orientation_tf_imu_, angular_velocity);
  angular_velocity_imu_ = tf2::toMsg(angular_velocity);

  time_imu_ = this->now();
}

void GnssToOdom::status_timer_callback()
{
  if ((this->now() - time_gnss_main_).seconds() > 1)
  {
    msg_status_.data |= (1ULL << uint8_t(LocStatus::error_no_signal_gnss_main));
  }
  else
  {
    msg_status_.data &= ~(1ULL << uint8_t(LocStatus::error_no_signal_gnss_main));
  }

  if ((this->now() - time_gnss_aux_).seconds() > 1)
  {
    msg_status_.data |= (1ULL << uint8_t(LocStatus::error_no_signal_gnss_aux));
  }
  else
  {
    msg_status_.data &= ~(1ULL << uint8_t(LocStatus::error_no_signal_gnss_aux));
  }

  if ((this->now() - time_imu_).seconds() > 1 && !disable_imu_)
  {
    msg_status_.data |= (1ULL << uint8_t(LocStatus::error_no_signal_imu));
  }
  else
  {
    msg_status_.data &= ~(1ULL << uint8_t(LocStatus::error_no_signal_imu));
  }

  if ((this->now() - time_rtcm_).seconds() > 5)
  {
    msg_status_.data |= (1ULL << uint8_t(LocStatus::error_no_rtcm));
  }
  else
  {
    msg_status_.data &= ~(1ULL << uint8_t(LocStatus::error_no_rtcm));
  }

  pub_status_->publish(msg_status_);
}
}  // namespace gnss_to_odom

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gnss_to_odom::GnssToOdom)
