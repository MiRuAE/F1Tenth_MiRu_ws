#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/odometry.hpp"
#include "vesc_msgs/msg/my_odom.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

class OdomNavigationNode : public rclcpp::Node
{
public:
  OdomNavigationNode()
  : Node("odom_navigation_node"),
    target_set_(false),
    goal_reached_(false)  // 초기에는 목표에 도달하지 않음
  {
    // 목표 좌표에 대한 기본값은 설정하지 않음.
    // 만약 파라미터를 선언하더라도, target_set_ 플래그로 실제 사용 여부를 결정합니다.
    this->declare_parameter<double>("target_x", 0.0);
    this->declare_parameter<double>("target_y", 0.0);
    // 나머지 파라미터는 기존과 동일하게 선언
    this->declare_parameter<double>("goal_tolerance", 0.05);
    this->declare_parameter<double>("max_speed", 1.0);
    this->declare_parameter<double>("kp_speed", 0.5);
    this->declare_parameter<double>("kp_steering", 1.0);
    // this->declare_parameter<double>("max_steer_angle", 0.34);
    // 기존에 선언된 파라미터들 외에 servo 관련 파라미터들을 선언합니다.
    this->declare_parameter<double>("servo_min", 0.3175);
    this->declare_parameter<double>("servo_max", 0.8405);
    this->declare_parameter<double>("steering_angle_to_servo_gain", -1.2135);
    this->declare_parameter<double>("steering_angle_to_servo_offset", 0.5650);

    // 파라미터 값은 초기값으로만 사용(초기에는 target_set_가 false이므로 무시됨)
    target_x_ = this->get_parameter("target_x").as_double();
    target_y_ = this->get_parameter("target_y").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();
    kp_speed_ = this->get_parameter("kp_speed").as_double();
    kp_steering_ = this->get_parameter("kp_steering").as_double();
    // max_steer_angle_ = this->get_parameter("max_steer_angle").as_double();
    // YAML 파일에서 값을 가져옵니다.
    double servo_min = this->get_parameter("servo_min").as_double();
    double servo_max = this->get_parameter("servo_max").as_double();
    double steering_gain = this->get_parameter("steering_angle_to_servo_gain").as_double();
    double steering_offset = this->get_parameter("steering_angle_to_servo_offset").as_double();

    // 계산 공식: steering_angle = (servo_value - offset) / gain
    // 여기서, 차량의 최대(최소) 조향각은 servo_min(servo_max)를 사용하여 계산합니다.
    max_steer_angle_ = (servo_min - steering_offset) / steering_gain; // 예: 약 0.2039 rad
    min_steer_angle_ = (servo_max - steering_offset) / steering_gain; // 예: 약 -0.2272 rad
    RCLCPP_INFO(this->get_logger(), "Computed max steer angle: %f rad, min steer angle: %f rad", max_steer_angle_, min_steer_angle_);

    RCLCPP_INFO(this->get_logger(), "No target set. Waiting for user input...");

    // AckermannDrive 명령 퍼블리셔 생성
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    // odom 토픽 서브스크라이버 생성
    odom_sub_ = this->create_subscription<vesc_msgs::msg::MyOdom>(
      "/odom", 10, std::bind(&OdomNavigationNode::odomCallback, this, _1));

    // 터미널 입력을 처리할 별도의 스레드를 생성하여 목표 좌표를 갱신
    input_thread_ = std::thread(&OdomNavigationNode::readTargetFromConsole, this);
  }

  ~OdomNavigationNode() override
  {
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
  }

private:
  // odom 콜백: target이 설정되어 있을 때만 주행 제어 수행
  void odomCallback(const vesc_msgs::msg::MyOdom::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (!target_set_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Target not set. Waiting for user input...");
      return;
    }

    // // 현재 위치
    // double x = msg->pose.pose.position.x;
    // double y = msg->pose.pose.position.y;

    // // 현재 자세 (쿼터니언을 오일러 각으로 변환)
    // tf2::Quaternion q;
    // tf2::fromMsg(msg->pose.pose.orientation, q);
    // double roll, pitch, current_yaw;
    // tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw);

    // MyOdom 메시지에서 직접 x, y, yaw 값을 사용
    double x = msg->x;
    double y = msg->y;
    double current_yaw = msg->yaw;

    // 목표 좌표와의 오차 계산
    double dx = target_x_ - x;
    double dy = target_y_ - y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // 목표에 도달한 경우 한 번만 메시지 출력
    if (distance < goal_tolerance_) {
      if (!goal_reached_) {
        stopVehicle();
        RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping.");
        goal_reached_ = true; // 한 번만 출력하도록 플래그 설정
      }
      return;
    } else {
      // 목표에 도달하지 않은 경우, 다시 주행 중이므로 플래그를 false로 유지
      goal_reached_ = false;
    }

    // std::atan2는 radian 단위를 반환하므로 도로 변환
    double target_heading = std::atan2(dy, dx) * 180.0 / M_PI;  // degree 단위
    double heading_error = target_heading - current_yaw;

    // heading error를 -180 ~ 180 도 범위로 정규화
    while (heading_error > 180.0)
      heading_error -= 360.0;
    while (heading_error < -180.0)
      heading_error += 360.0;

    // 간단한 P 제어를 이용한 속도 계산
    double speed_cmd = kp_speed_ * distance;
    if (speed_cmd > max_speed_) {
      speed_cmd = max_speed_;
    }

    // 간단한 P 제어를 이용한 조향각 계산
    double steering_cmd = kp_steering_ * heading_error;
    // 조향각 명령을 radian 단위로 변환 (1 degree = M_PI/180 rad)
    double steering_cmd_rad = steering_cmd * M_PI / 180.0;
    if (steering_cmd_rad > max_steer_angle_) {
      steering_cmd_rad = max_steer_angle_;
    } else if (steering_cmd_rad < min_steer_angle_) {
      steering_cmd_rad = min_steer_angle_;
    }
    // if (steering_cmd_rad > max_steer_angle_) {
    //   steering_cmd_rad = max_steer_angle_;
    // } else if (steering_cmd_rad < -max_steer_angle_) {
    //   steering_cmd_rad = -max_steer_angle_;
    // }

    // AckermannDriveStamped 메시지 생성 및 퍼블리시
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = this->get_clock()->now();
    drive_msg.drive.speed = speed_cmd;
    drive_msg.drive.steering_angle = steering_cmd_rad;
    drive_pub_->publish(drive_msg);
  }

  void stopVehicle()
  {
    auto stop_msg = ackermann_msgs::msg::AckermannDriveStamped();
    stop_msg.header.stamp = this->get_clock()->now();
    stop_msg.drive.speed = 0.0;
    stop_msg.drive.steering_angle = 0.0;
    drive_pub_->publish(stop_msg);
  }

  // 터미널 입력을 읽어 목표 좌표를 갱신하는 함수 (별도 스레드에서 실행)
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
        goal_reached_ = false; // 새 목표를 설정하면 goal_reached_ 초기화
      }
      RCLCPP_INFO(this->get_logger(), "New target set to (x: %f, y: %f)", target_x_, target_y_);
    }
  }

  // 멤버 변수
  rclcpp::Subscription<vesc_msgs::msg::MyOdom>::SharedPtr odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

  double target_x_;
  double target_y_;
  bool target_set_;
  bool goal_reached_;  // 목표에 도달했는지 여부를 저장하는 플래그
  double goal_tolerance_;
  double max_speed_;
  double kp_speed_;
  double kp_steering_;
  double max_steer_angle_;
  double min_steer_angle_;

  std::thread input_thread_;
  std::mutex target_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomNavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}