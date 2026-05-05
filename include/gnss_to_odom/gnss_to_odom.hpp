// Copyright 2026 SABI AGRI

#ifndef GNSS_TO_ODOM__GNSS_TO_ODOM_HPP_
#define GNSS_TO_ODOM__GNSS_TO_ODOM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "GeographicLib/LocalCartesian.hpp"
#include "bondcpp/bond.hpp"

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace gnss_to_odom
{
enum class LocStatus : uint8_t
{
  // Info status [0-15] bits
  info_normal_operation = 0,
  // Warn status [16-31] bits
  // Error status [32-63] bits
  error_no_signal_gnss_aux = 32,
  error_no_signal_gnss_main = 33,
  error_no_signal_imu = 34,
  error_no_rtk_fix = 35,
  error_covariance = 36,
  error_check_accuracy = 37,
};

class GnssToOdom : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit GnssToOdom(const rclcpp::NodeOptions& options);

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

protected:
  void fix_to_odom(const gps_msgs::msg::GPSFix::SharedPtr& fix, nav_msgs::msg::Odometry& odom);
  void wgs84_to_enu(double latitude, double longitude, double altitude, geometry_msgs::msg::Point& position);
  void update_odometry();
  bool check_accuracy();
  void publish();

  void gnss_main_callback(const gps_msgs::msg::GPSFix::SharedPtr msg);
  void gnss_aux_callback(const gps_msgs::msg::GPSFix::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void status_timer_callback();

private:
  bool publish_tf_{ false };
  bool disable_imu_{ false };
  std::string frame_gnss_main_{ "main_gnss" };
  std::string frame_gnss_aux_{ "aux_gnss" };
  std::string frame_imu_{ "imu" };
  double covariance_threshold_{ 0.01 };
  double antenna_distance_tolerance_{ 0.05 };

  double yaw_offset_;
  int16_t gnss_main_status_;
  int16_t gnss_aux_status_;
  int8_t odom_speed_sign_{ 1 };
  bool enu_initialized_{ false };

  rclcpp::Time time_imu_;
  rclcpp::Time time_gnss_main_;
  rclcpp::Time time_gnss_aux_;
  GeographicLib::LocalCartesian enu_;

  tf2::Vector3 position_tf_gnss_;
  tf2::Vector3 position_tf_gnss_main_;
  tf2::Vector3 position_tf_gnss_aux_;
  tf2::Quaternion orientation_tf_imu_;
  geometry_msgs::msg::Vector3 orientation_imu_;
  geometry_msgs::msg::Vector3 angular_velocity_imu_;

  geometry_msgs::msg::TransformStamped tf_odom_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  std::unique_ptr<bond::Bond> bond_;

  nav_msgs::msg::Odometry msg_odom_;
  nav_msgs::msg::Odometry msg_odom_main_;
  nav_msgs::msg::Odometry msg_odom_aux_;
  std_msgs::msg::UInt64 msg_status_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_main_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_aux_;
  rclcpp_lifecycle::LifecyclePublisher<gps_msgs::msg::GPSFix>::SharedPtr pub_odom_wgs84_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt64>::SharedPtr pub_status_;

  rclcpp::TimerBase::SharedPtr timer_status_;

  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr sub_gnss_main_;
  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr sub_gnss_aux_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
};
}  // namespace gnss_to_odom

#endif  // GNSS_TO_ODOM__GNSS_TO_ODOM_HPP_
