// Copyright 2020 F1TENTH Foundation
// Modified to include IMU-based filtering for improved odometry accuracy

#include "vesc_ackermann/vesc_to_odom.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include <string>

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

class VescToOdomWithIMU : public rclcpp::Node
{
public:
  VescToOdomWithIMU(const rclcpp::NodeOptions & options)
  : Node("vesc_to_odom_with_imu_node", options),
    x_(0.0), y_(0.0), yaw_(0.0), imu_yaw_rate_(0.0), last_imu_time_(this->now())
  {
    // Get ROS parameters
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_link");
    use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", true);
    publish_tf_ = declare_parameter("publish_tf", false);
    
    speed_to_erpm_gain_ = declare_parameter("speed_to_erpm_gain", 0.0);
    speed_to_erpm_offset_ = declare_parameter("speed_to_erpm_offset", 0.0);
    wheelbase_ = declare_parameter("wheelbase", 0.0);

    // Publishers and Subscribers
    odom_pub_ = create_publisher<Odometry>("odom", 10);
    vesc_state_sub_ = create_subscription<VescStateStamped>(
      "sensors/core", 10, std::bind(&VescToOdomWithIMU::vescStateCallback, this, _1));
    imu_sub_ = create_subscription<Imu>(
      "imu/data", 10, std::bind(&VescToOdomWithIMU::imuCallback, this, _1));
    if (use_servo_cmd_) {
      servo_sub_ = create_subscription<Float64>(
        "sensors/servo_position_command", 10, std::bind(&VescToOdomWithIMU::servoCmdCallback, this, _1));
    }
  }

private:
  void vescStateCallback(const VescStateStamped::SharedPtr state)
  {
    if (use_servo_cmd_ && !last_servo_cmd_) return;

    double current_speed = (-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
    if (std::fabs(current_speed) < 0.05) current_speed = 0.0;

    double current_steering_angle = use_servo_cmd_ ? (last_servo_cmd_->data / steering_to_servo_gain_) : 0.0;
    double current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;

    if (!last_state_) last_state_ = state;
    auto dt = (rclcpp::Time(state->header.stamp) - rclcpp::Time(last_state_->header.stamp)).seconds();

    // Update position using filtered yaw
    double x_dot = current_speed * cos(yaw_);
    double y_dot = current_speed * sin(yaw_);
    x_ += x_dot * dt;
    y_ += y_dot * dt;

    // Apply complementary filter for yaw angle
    double filtered_yaw = complementaryFilter(yaw_, imu_yaw_rate_, dt);

    yaw_ = filtered_yaw;

    last_state_ = state;
    publishOdometry(state->header.stamp, current_speed, current_angular_velocity);
  }

  void imuCallback(const Imu::SharedPtr imu)
  {
    imu_yaw_rate_ = imu->angular_velocity.z;
  }

  void servoCmdCallback(const Float64::SharedPtr servo)
  {
    last_servo_cmd_ = servo;
  }

  void publishOdometry(const rclcpp::Time & timestamp, double speed, double angular_velocity)
  {
    Odometry odom;
    odom.header.frame_id = odom_frame_;
    odom.header.stamp = timestamp;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
    odom.pose.pose.orientation.w = cos(yaw_ / 2.0);

    odom.twist.twist.linear.x = speed;
    odom.twist.twist.angular.z = angular_velocity;

    if (rclcpp::ok()) {
      odom_pub_->publish(odom);
    }
  }

  // Complementary filter to combine IMU and wheel odometry
  double complementaryFilter(double previous_yaw, double imu_rate, double dt)
  {
    // Time constant for the complementary filter (choose this based on your system's dynamics)
    double alpha = 0.98;  // A typical value is around 0.98, but you may need to fine-tune it
    
    // Integrate the IMU yaw rate to get the new yaw value
    double imu_yaw = previous_yaw + imu_rate * dt;

    // Combine with the previous yaw (from wheel odometry, for example) using the complementary filter
    return alpha * imu_yaw + (1.0 - alpha) * previous_yaw;
  }

  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::Subscription<Float64>::SharedPtr servo_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;

  std::string odom_frame_, base_frame_;
  bool use_servo_cmd_, publish_tf_;
  double speed_to_erpm_gain_, speed_to_erpm_offset_, wheelbase_;
  double x_, y_, yaw_, imu_yaw_rate_;
  VescStateStamped::SharedPtr last_state_;
  Float64::SharedPtr last_servo_cmd_;
  rclcpp::Time last_imu_time_;
};

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdomWithIMU)
