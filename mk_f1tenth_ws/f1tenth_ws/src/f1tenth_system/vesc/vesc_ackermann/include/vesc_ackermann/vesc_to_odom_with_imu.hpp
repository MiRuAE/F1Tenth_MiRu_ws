// Copyright 2020 F1TENTH Foundation
// Modified to include IMU-based filtering for improved odometry accuracy

#ifndef VESC_ACKERMANN__VESC_TO_ODOM_WITH_IMU_HPP_
#define VESC_ACKERMANN__VESC_TO_ODOM_WITH_IMU_HPP_

// #include <nav_msgs/msg/odometry.hpp>
#include <vesc_msgs/msg/my_odom.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
// #include <sensor_msgs/msg/imu.hpp>
#include <vesc_msgs/msg/vesc_imu_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <memory>
#include <string>

namespace vesc_ackermann
{

// using nav_msgs::msg::Odometry;
using vesc_msgs::msg::MyOdom;
using std_msgs::msg::Float64;
// using sensor_msgs::msg::Imu;
using vesc_msgs::msg::VescImuStamped;
using vesc_msgs::msg::VescStateStamped;

class VescToOdomWithIMU : public rclcpp::Node
{
public:
  explicit VescToOdomWithIMU(const rclcpp::NodeOptions & options);

private:
  // Callback functions
  void vescStateCallback(const VescStateStamped::SharedPtr state);
  void imuCallback(const VescImuStamped::SharedPtr imu);
  void servoCmdCallback(const Float64::SharedPtr servo);
  void publishOdometry(const rclcpp::Time & timestamp, double speed, double angular_velocity);

  // Complementary filter to combine IMU and wheel odometry data
  double complementaryFilter(double previous_yaw, double imu_rate, double dt);

  // ROS parameters and internal state
  std::string odom_frame_;
  std::string base_frame_;
  bool use_servo_cmd_;
  bool publish_tf_;
  double speed_to_erpm_gain_;
  double speed_to_erpm_offset_;
  double wheelbase_;
  double steering_to_servo_gain_;

  // Odometry state
  double x_;
  double y_;
  double yaw_;
  double imu_yaw_rate_;

  // Last received messages
  VescStateStamped::SharedPtr last_state_;
  Float64::SharedPtr last_servo_cmd_;

  // ROS publishers and subscribers
  rclcpp::Publisher<MyOdom>::SharedPtr odom_pub_;
  rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::Subscription<Float64>::SharedPtr servo_sub_;
  rclcpp::Subscription<VescImuStamped>::SharedPtr imu_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__VESC_TO_ODOM_WITH_IMU_HPP_