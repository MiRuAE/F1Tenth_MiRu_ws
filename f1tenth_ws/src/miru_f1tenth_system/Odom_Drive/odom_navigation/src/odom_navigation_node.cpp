#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "odom_msgs/msg/my_odom.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class OdomNavigationNode : public rclcpp::Node
{
public:
  OdomNavigationNode()
  : Node("odom_navigation_node"),
    target_set_(false),
    goal_reached_(false),
    is_active_(false),
    origin_set_(false),
    prev_cte_(0.0),
    integral_cte_(0.0),
    prev_speed_error_(0.0),
    integral_speed_error_(0.0)
  {
    // Mission state subscriber
    mission_sub_ = this->create_subscription<std_msgs::msg::String>(
      "current_mission", 10,
      std::bind(&OdomNavigationNode::mission_callback, this, std::placeholders::_1));

    // Subscribe to transition position from Mission B
    transition_sub_ = this->create_subscription<odom_msgs::msg::MyOdom>(
      "mission_b_end_position", 10,
      std::bind(&OdomNavigationNode::transition_callback, this, std::placeholders::_1));

    // 기존 목표 좌표 및 속도 관련 파라미터
    this->declare_parameter<double>("target_x", 0.0);
    this->declare_parameter<double>("target_y", 0.0);
    this->declare_parameter<double>("goal_tolerance", 0.05);

    // Servo 관련 파라미터 (조향각 제한 계산용)
    this->declare_parameter<double>("servo_min", 0.3175);
    this->declare_parameter<double>("servo_max", 0.8405);
    this->declare_parameter<double>("steering_angle_to_servo_gain", -1.2135);
    this->declare_parameter<double>("steering_angle_to_servo_offset", 0.5650);

    // Pure Pursuit 관련 파라미터
    this->declare_parameter<double>("lookahead_distance", 0.5);  // Lookahead 거리 (미터)
    this->declare_parameter<double>("wheelbase", 0.32);          // 차량 휠베이스 (미터)

    // Steering 제어 PID 파라미터
    this->declare_parameter<double>("kp_pid", 0.0);
    this->declare_parameter<double>("ki_pid", 0.0);
    this->declare_parameter<double>("kd_pid", 0.0);

    // 속도 제어 PID 파라미터 및 감속 구간 설정
    this->declare_parameter<double>("kp_speed_pid", 0.5);
    this->declare_parameter<double>("ki_speed_pid", 0.0);
    this->declare_parameter<double>("kd_speed_pid", 0.0);
    // 목표와 가까워졌을 때 감속을 위한 거리 (미터)
    this->declare_parameter<double>("decel_distance", 0.5);

    // 최소/최대 속도 값
    this->declare_parameter<double>("max_speed", 2.5);
    this->declare_parameter<double>("min_speed", 0.5);

    // 파라미터 값 초기화
    target_x_ = this->get_parameter("target_x").as_double();
    target_y_ = this->get_parameter("target_y").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();

    kp_pid_ = this->get_parameter("kp_pid").as_double();
    ki_pid_ = this->get_parameter("ki_pid").as_double();
    kd_pid_ = this->get_parameter("kd_pid").as_double();

    kp_speed_pid_ = this->get_parameter("kp_speed_pid").as_double();
    ki_speed_pid_ = this->get_parameter("ki_speed_pid").as_double();
    kd_speed_pid_ = this->get_parameter("kd_speed_pid").as_double();
    decel_distance_ = this->get_parameter("decel_distance").as_double();

    max_speed_ = this->get_parameter("max_speed").as_double();
    min_speed_ = this->get_parameter("min_speed").as_double();

    // Servo 파라미터를 이용해 조향각 제한 계산 (라디안 단위)
    double servo_min = this->get_parameter("servo_min").as_double();
    double servo_max = this->get_parameter("servo_max").as_double();
    double steering_gain = this->get_parameter("steering_angle_to_servo_gain").as_double();
    double steering_offset = this->get_parameter("steering_angle_to_servo_offset").as_double();
    max_steer_angle_ = (servo_min - steering_offset) / steering_gain; // 예: 약 0.2039 rad
    min_steer_angle_ = (servo_max - steering_offset) / steering_gain; // 예: 약 -0.2272 rad
    RCLCPP_INFO(this->get_logger(), "Computed max steer angle: %f rad, min steer angle: %f rad", max_steer_angle_, min_steer_angle_);

    RCLCPP_INFO(this->get_logger(), "No target set. Waiting for user input...");

    // AckermannDrive 명령 퍼블리셔 생성
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    // odom 토픽 서브스크라이버 생성
    odom_sub_ = this->create_subscription<odom_msgs::msg::MyOdom>(
      "/my_odom", 10, std::bind(&OdomNavigationNode::odomCallback, this, _1));

    // 터미널 입력을 처리할 별도 스레드 생성 (목표 좌표 갱신)
    input_thread_ = std::thread(&OdomNavigationNode::readTargetFromConsole, this);
    
    // 🔹 파라미터 변경 콜백 등록
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&OdomNavigationNode::onParameterChange, this, std::placeholders::_1));

    // 로그 출력
    RCLCPP_INFO(this->get_logger(), "PID 초기값: Kp = %f, Ki = %f, Kd = %f", kp_pid_, ki_pid_, kd_pid_);
  }

  ~OdomNavigationNode() override
  {
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
  }

private:
  void mission_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    bool was_active = is_active_;
    is_active_ = (msg->data == "MISSION_C");
    
    if (is_active_ != was_active) {
      if (is_active_) {
        RCLCPP_INFO(this->get_logger(), "Odometry node activated - Mission C");
        // Reset origin tracking when entering Mission C
        origin_set_ = false;
      } else {
        RCLCPP_INFO(this->get_logger(), "Odometry node deactivated");
        // Clear origin when leaving Mission C
        origin_set_ = false;
      }
    }
  }

  void transition_callback(const odom_msgs::msg::MyOdom::SharedPtr msg)
  {
    if (!is_active_) return;  // Only process if we're in Mission C
    
    // Get the position from the transition message using our custom message format
    double transition_x = msg->x;
    double transition_y = msg->y;
    double transition_yaw = msg->yaw;  // Store the yaw for potential use in navigation
    
    // Add offset to the transition position (configurable via parameters)
    double offset_x = this->declare_parameter("target_offset_x", 0.8);  // Default 0.8m forward
    double offset_y = this->declare_parameter("target_offset_y", 0.5);  // Default 0.5m to the right
    
    // Convert the offset from local coordinates (relative to car) to global coordinates
    double yaw_rad = transition_yaw * M_PI / 180.0;  // Convert yaw to radians if it's in degrees
    double global_offset_x = offset_x * std::cos(yaw_rad) - offset_y * std::sin(yaw_rad);
    double global_offset_y = offset_x * std::sin(yaw_rad) + offset_y * std::cos(yaw_rad);
    
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      target_x_ = transition_x + global_offset_x;
      target_y_ = transition_y + global_offset_y;
      target_set_ = true;
      goal_reached_ = false;
      
      // Reset PID variables for the new target
      prev_cte_ = 0.0;
      integral_cte_ = 0.0;
      prev_speed_error_ = 0.0;
      integral_speed_error_ = 0.0;
      
      RCLCPP_INFO(this->get_logger(), 
        "New target set from transition position (%.2f, %.2f, yaw: %.2f) with offset (%.2f, %.2f) -> target: (%.2f, %.2f)", 
        transition_x, transition_y, transition_yaw,
        offset_x, offset_y, target_x_, target_y_);
    }
  }

  void odomCallback(const odom_msgs::msg::MyOdom::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    
    // Set origin position when first entering Mission C
    if (is_active_ && !origin_set_) {
      origin_x_ = msg->x;
      origin_y_ = msg->y;
      origin_yaw_ = msg->yaw;
      origin_set_ = true;
      RCLCPP_INFO(this->get_logger(), "Set origin position for Mission C at (%.2f, %.2f, yaw: %.2f)",
                  origin_x_, origin_y_, origin_yaw_);
    }

    if (!target_set_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Target not set. Waiting for user input...");
      return;
    }

    // Calculate relative position to origin
    double x_rel = msg->x - origin_x_;
    double y_rel = msg->y - origin_y_;
    double current_yaw = msg->yaw;
    double current_yaw_rad = current_yaw * M_PI / 180;

    // Rotate coordinates based on origin yaw to get relative coordinates
    double origin_yaw_rad = origin_yaw_ * M_PI / 180.0;
    double x = std::cos(-origin_yaw_rad) * x_rel - std::sin(-origin_yaw_rad) * y_rel;
    double y = std::sin(-origin_yaw_rad) * x_rel + std::cos(-origin_yaw_rad) * y_rel;

    // 목표 좌표와의 거리 계산 (target_x_, target_y_는 이미 상대 좌표)
    double dx = target_x_ - x;
    double dy = target_y_ - y;
    double distance = std::sqrt(dx * dx + dy * dy);

    RCLCPP_INFO(this->get_logger(), "Distance to goal: %f, Goal tolerance: %f", distance, goal_tolerance_);
    // 목표 도달 체크
    if (distance < goal_tolerance_) {
      if (!goal_reached_) {
        stopVehicle();
        RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping.");
        goal_reached_ = true;
      }
      return;
    } else {
      goal_reached_ = false;
    }

    // Only proceed with calculations and publish drive command if in Mission C
    if (!is_active_) {
      return;
    }

    // 목표 방향 (라디안)
    double target_heading = std::atan2(dy, dx);

    // Lookahead Point 결정
    double effective_lookahead = lookahead_distance_;
    double lookahead_x, lookahead_y;
    if(distance < lookahead_distance_) {
      lookahead_x = target_x_;
      lookahead_y = target_y_;
      effective_lookahead = distance;  // 남은 거리를 사용
    } else {
      lookahead_x = x + lookahead_distance_ * std::cos(target_heading);
      lookahead_y = y + lookahead_distance_ * std::sin(target_heading);
    }

    // 현재 위치 기준 Lookahead Point의 상대 좌표 (로컬 좌표계)
    double rel_x = lookahead_x - x;
    double rel_y = lookahead_y - y;
    double transformed_x = std::cos(-current_yaw_rad) * rel_x - std::sin(-current_yaw_rad) * rel_y;
    double transformed_y = std::sin(-current_yaw_rad) * rel_x + std::cos(-current_yaw_rad) * rel_y;

    // Pure Pursuit 조향각 계산
    double alpha = std::atan2(transformed_y, transformed_x);
    double pure_pursuit_steer = std::atan2(2.0 * wheelbase_ * std::sin(alpha), effective_lookahead);

    // Steering PID 보정 (로컬 좌표계 y값을 오차로 사용)
    double cte = transformed_y;
    double diff_cte = cte - prev_cte_;
    integral_cte_ += cte;
    double pid_correction = kp_pid_ * cte + ki_pid_ * integral_cte_ + kd_pid_ * diff_cte;
    prev_cte_ = cte;

    // 최종 조향각 = Pure Pursuit 조향각 + PID 보정
    double steering_cmd_rad = pure_pursuit_steer + pid_correction;
    if (steering_cmd_rad > max_steer_angle_) {
      steering_cmd_rad = max_steer_angle_;
    } else if (steering_cmd_rad < min_steer_angle_) {
      steering_cmd_rad = min_steer_angle_;
    }

    // =====================
    // 속도 제어 (PID 적용)
    // =====================
    // deceleration 구간 내에서는 목표 속도를 선형 보간하여 낮춤
    double desired_speed;
    if(distance < decel_distance_) {
      desired_speed = min_speed_ + (max_speed_ - min_speed_) * (distance / decel_distance_);
    } else {
      desired_speed = max_speed_;
    }
    // 현재 속도 (MyOdom 메시지에 velocity 필드가 있다고 가정)
    double current_speed = msg->linear_velocity;
    double speed_error = desired_speed - current_speed;
    double diff_speed_error = speed_error - prev_speed_error_;
    integral_speed_error_ += speed_error;
    double speed_cmd = kp_speed_pid_ * speed_error +
                       ki_speed_pid_ * integral_speed_error_ +
                       kd_speed_pid_ * diff_speed_error;
    prev_speed_error_ = speed_error;
    // 속도 명령을 최소/최대 범위 내로 제한
    if (speed_cmd > max_speed_) {
      speed_cmd = max_speed_;
    } else if (speed_cmd < min_speed_) {
      speed_cmd = min_speed_;
    }

    // AckermannDriveStamped 메시지 생성 및 퍼블리시
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = this->get_clock()->now();
    drive_msg.drive.speed = speed_cmd;
    drive_msg.drive.steering_angle = steering_cmd_rad;
    drive_pub_->publish(drive_msg);
  }

  void stopVehicle()
  {
    RCLCPP_INFO(this->get_logger(), "stopVehicle() called! Publishing stop command.");
    auto stop_msg = ackermann_msgs::msg::AckermannDriveStamped();
    stop_msg.header.stamp = this->get_clock()->now();
    stop_msg.drive.speed = 0.0;
    stop_msg.drive.steering_angle = 0.0;
    drive_pub_->publish(stop_msg);
  }

  // 터미널 입력을 통해 목표 좌표를 갱신 (별도 스레드)
  void readTargetFromConsole()
  {
    while (rclcpp::ok()) {
      std::cout << "Enter target_x and target_y separated by space (or 'q' to quit): ";
      std::string line;
      std::getline(std::cin, line);
      if (line == "q") {
        rclcpp::shutdown();
        break;
      }
      std::istringstream iss(line);
      double new_target_x, new_target_y;
      if (!(iss >> new_target_x >> new_target_y)) {
        std::cout << "Invalid input. Please enter two numbers." << std::endl;
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_x_ = new_target_x;
        target_y_ = new_target_y;
        target_set_ = true;
        goal_reached_ = false;
        // PID 관련 변수 초기화 (새 목표 설정 시)
        prev_cte_ = 0.0;
        integral_cte_ = 0.0;
        prev_speed_error_ = 0.0;
        integral_speed_error_ = 0.0;
      }
      RCLCPP_INFO(this->get_logger(), "New target set to (x: %f, y: %f)", target_x_, target_y_);
    }
  }
  
  // 🔹 파라미터 변경 감지 콜백 함수
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    for (const auto &param : parameters)
    {
      if (param.get_name() == "kp_pid") {
        kp_pid_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Kp 변경됨: %f", kp_pid_);
      }
      else if (param.get_name() == "ki_pid") {
        ki_pid_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Ki 변경됨: %f", ki_pid_);
      }
      else if (param.get_name() == "kd_pid") {
        kd_pid_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Kd 변경됨: %f", kd_pid_);
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // 멤버 변수
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Subscription<odom_msgs::msg::MyOdom>::SharedPtr transition_sub_;
  bool is_active_;
  rclcpp::Subscription<odom_msgs::msg::MyOdom>::SharedPtr odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;


  double target_x_;
  double target_y_;
  bool target_set_;
  bool goal_reached_;
  double goal_tolerance_;

  // 속도 관련 변수
  double max_speed_;
  double min_speed_;

  // Servo 파라미터로부터 계산된 조향각 제한 (radian 단위)
  double max_steer_angle_;
  double min_steer_angle_;

  // Pure Pursuit 관련
  double lookahead_distance_;
  double wheelbase_;

  // Steering PID 제어 파라미터 및 변수
  double kp_pid_, ki_pid_, kd_pid_;
  double prev_cte_;
  double integral_cte_;

  // 속도 PID 제어 파라미터 및 변수
  double kp_speed_pid_, ki_speed_pid_, kd_speed_pid_;
  double decel_distance_;
  double prev_speed_error_;
  double integral_speed_error_;

  std::thread input_thread_;
  std::mutex target_mutex_;

  // Add new member variables for origin tracking
  bool origin_set_;
  double origin_x_;
  double origin_y_;
  double origin_yaw_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomNavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
