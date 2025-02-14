#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node"), speed_(0.0)
    {
        // LaserScan 메시지를 구독하여 scan_callback 호출
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));

        // Odometry 메시지를 구독하여 odom_callback 호출
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ego_racecar/odom", 10, std::bind(&Safety::odom_callback, this, std::placeholders::_1));

        // AckermannDriveStamped 메시지를 발행하는 퍼블리셔 생성
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        /*drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/teleop", 10);*/

    }
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers
        
    

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    double speed_;
    // TODO: create ROS subscribers and publishers

    // void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    // {
    //     /// TODO: update current speed
    // }

    // void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    // {
    //     /// TODO: calculate TTC

    //     /// TODO: publish drive/brake message
    // }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 현재 속도 저장 (x축 속도)
        speed_ = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Time-to-Collision (TTC) 계산
        double min_ttc = std::numeric_limits<double>::max();

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            double distance = msg->ranges[i];
            double angle = msg->angle_min + i * msg->angle_increment;
            double relative_speed = speed_ * cos(angle);

            if (relative_speed > 0) { // 충돌 가능성이 있는 경우만 계산
                double ttc = distance / relative_speed;
                if (ttc < min_ttc) {
                    min_ttc = ttc;
                }
            }
        }

        // 일정 임계값(TTC_THRESHOLD) 이하이면 긴급 제동
        constexpr double TTC_THRESHOLD = 1.5; // 0.5초 이하일 경우 긴급 제동
        if (min_ttc < TTC_THRESHOLD) {
            RCLCPP_WARN(this->get_logger(), "Emergency Brake Activated! TTC: %f", min_ttc);
            brake();
        }
    }

    void brake() {
        // 제동 명령 생성
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = 0.0; // 속도를 0으로 설정 (긴급 정지)
        drive_pub_->publish(drive_msg);
    }



};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
