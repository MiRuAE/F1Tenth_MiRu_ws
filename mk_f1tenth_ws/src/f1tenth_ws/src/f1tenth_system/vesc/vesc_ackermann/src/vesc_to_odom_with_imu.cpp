// Copyright 2020 F1TENTH Foundation
// Modified to include IMU-based filtering for improved odometry accuracy

#include "vesc_ackermann/vesc_to_odom_with_imu.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <string>

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
// using nav_msgs::msg::Odometry;
using vesc_msgs::msg::MyOdom;

// using sensor_msgs::msg::Imu;
using vesc_msgs::msg::VescImuStamped;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

VescToOdomWithIMU::VescToOdomWithIMU(const rclcpp::NodeOptions & options)
: Node("vesc_to_odom_with_imu_node", options),
  x_(0.0), y_(0.0), yaw_(0.0), imu_yaw_rate_(0.0)
{
  // Get ROS parameters
  odom_frame_ = declare_parameter("odom_frame", "odom");
  base_frame_ = declare_parameter("base_frame", "base_link");
  use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", true);
  publish_tf_ = declare_parameter("publish_tf", false);
  
  speed_to_erpm_gain_ = declare_parameter("speed_to_erpm_gain", 0.0);
  speed_to_erpm_offset_ = declare_parameter("speed_to_erpm_offset", 0.0);
  wheelbase_ = declare_parameter("wheelbase", 0.0);
  steering_to_servo_gain_ = declare_parameter("steering_angle_to_servo_gain", 0.0);

  // Publishers and Subscribers
  odom_pub_ = create_publisher<MyOdom>("odom", 10);
  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", 10, std::bind(&VescToOdomWithIMU::vescStateCallback, this, _1));
  imu_sub_ = create_subscription<VescImuStamped>(
    "sensors/imu", 10, std::bind(&VescToOdomWithIMU::imuCallback, this, _1));
  if (use_servo_cmd_) {
    servo_sub_ = create_subscription<Float64>(
      "sensors/servo_position_command", 10, std::bind(&VescToOdomWithIMU::servoCmdCallback, this, _1));
  }
}

void VescToOdomWithIMU::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  if (use_servo_cmd_ && !last_servo_cmd_) return;

  double current_speed = (-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05)
    current_speed = 0.0;

  double current_steering_angle = use_servo_cmd_ ? (last_servo_cmd_->data / steering_to_servo_gain_) : 0.0;
  double current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;

  if (!last_state_) {
    last_state_ = state;
  }
  auto dt = (rclcpp::Time(state->header.stamp) - rclcpp::Time(last_state_->header.stamp)).seconds();

  // Update position using current yaw value
  double x_dot = current_speed * cos(yaw_);
  double y_dot = current_speed * sin(yaw_);
  x_ += x_dot * dt;
  y_ += y_dot * dt;

  // Apply complementary filter for yaw angle using IMU data
  double filtered_yaw = complementaryFilter(yaw_, imu_yaw_rate_, dt);
  yaw_ = filtered_yaw;

  last_state_ = state;
  publishOdometry(state->header.stamp, current_speed, current_angular_velocity);
}

void VescToOdomWithIMU::imuCallback(const VescImuStamped::SharedPtr imu)
{
  imu_yaw_rate_ = imu->imu.angular_velocity.z;  // IMU yaw rate in rad/s
}

void VescToOdomWithIMU::servoCmdCallback(const Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
}

void VescToOdomWithIMU::publishOdometry(const rclcpp::Time & timestamp, double speed, double angular_velocity)
{
  // Odometry odom;
  // odom.header.frame_id = odom_frame_;
  // odom.header.stamp = timestamp;
  // odom.child_frame_id = base_frame_;

  // // Set position and orientation
  // odom.pose.pose.position.x = x_;
  // odom.pose.pose.position.y = y_;
  // odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
  // odom.pose.pose.orientation.w = cos(yaw_ / 2.0);

  // // Set velocity (linear and angular)
  // odom.twist.twist.linear.x = speed;
  // odom.twist.twist.angular.z = angular_velocity;

  // if (rclcpp::ok()) {
  //   odom_pub_->publish(odom);
  // }

  auto odom_msg = vesc_msgs::msg::MyOdom();
  
  odom_msg.header.stamp = timestamp;
  odom_msg.header.frame_id = odom_frame_;
  
  odom_msg.x = x_;
  odom_msg.y = y_;
  odom_msg.yaw = yaw_;
  odom_msg.linear_velocity = speed;
  odom_msg.angular_velocity = angular_velocity;
  
  odom_pub_->publish(odom_msg);
}

double VescToOdomWithIMU::complementaryFilter(double previous_yaw, double imu_rate, double dt)
{
  // Time constant for the complementary filter (tunable parameter)
  double alpha = 0.98;
  double imu_yaw = previous_yaw + imu_rate * dt;
  return alpha * imu_yaw + (1.0 - alpha) * previous_yaw;
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdomWithIMU)