#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers

        // LaserScan 메시지를 구독하여 scan_callback 호출
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));

        // Odometry 메시지를 구독하여 odom_callback 호출
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&WallFollow::odom_callback, this, std::placeholders::_1));

        // AckermannDriveStamped 메시지를 발행하는 퍼블리셔 생성
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    }

private:
    // PID CONTROL PARAMS
    // TODO: double kp =
    // TODO: double kd =
    // TODO: double ki =
    double servo_offset = 0.0;
    /*double prev_error = 0.0;*/
    double error = 0.0;
    double integral = 0.0;

    double kp = 3.0;
    double ki = 0.001;
    double kd = 0.05;

    double ref_angle_f = M_PI / 6;
    double ref_angle_r = M_PI / 2;
    double ref_dist = 0.9;

    double speed_ = 0.0;
    double scan_delay = 0.004;

    // Topics
    /*std::string lidarscan_topic = "/scan";*/
    /*std::string drive_topic = "/drive";*/
    /// TODO: create ROS subscribers and publishers

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    double get_range(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // TODO: implement
        
        int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
        double range = scan_msg->ranges[index];

        return range;
    }

    double get_error(float range_f, float range_r, double ref_dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // TODO:implement

        double theta = ref_angle_r - ref_angle_f;
        double numerator = range_f * std::cos(theta) - range_r;
        double denominator = range_f * std::sin(theta);
        double error_angle = std::atan2(numerator, denominator);
        double present_dist = range_r * std::cos(error_angle);
        double future_dist = present_dist + (speed_ * scan_delay) *                               std::sin(error_angle);
        double error_dist = -(ref_dist - future_dist);

        return error_dist;

    }
    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        static double prev_error = 0.0;
        static rclcpp::Time pid_last_time = this->now();    // save last time
        rclcpp::Time current_time = this->now();

        double angle = 0.0;
        // TODO: Use kp, ki & kd to implement a PID controller
        /*double dt = (current_time - pid_last_time).seconds();*/
        double dt = std::max(0.01, (current_time - pid_last_time).seconds());
        double integral_error = 0.0;
        integral_error += (prev_error + error) / 2.0 * dt;
        double derivative_error = (error - prev_error) / dt;
        prev_error = error;
        pid_last_time = current_time;
        
        double pid_result = kp * error +
                            ki * integral_error +
                            kd * derivative_error;
        
    /*If the steering angle is between 0 degrees and 10 degrees, the car should drive at 1.5 meters per second.*/
    /*If the steering angle is between 10 degrees and 20 degrees, the speed should be 1.0 meters per second.*/
    /*Otherwise, the speed should be 0.5 meters per second.*/
        double abs_pid_result = std::abs(pid_result);
        double drive_speed = 0.0;

        if (abs_pid_result <= 10.0) {
          drive_speed = 1.2;
        } else if (abs_pid_result <= 20.0) {
          drive_speed = 0.8;
        } else {
          drive_speed = 0.3;
        }

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
        drive_msg.drive.steering_angle = pid_result;
        drive_msg.drive.speed = drive_speed;
        
        drive_pub_->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        double error = 0.0; // TODO: replace with error calculated by get_error()
        double velocity = 0.0; // TODO: calculate desired car velocity based on error
        // TODO: actuate the car with PID


        double range_f = get_range(scan_msg, ref_angle_f);
        double range_r = get_range(scan_msg, ref_angle_r);

        error = get_error(range_f, range_r, ref_dist);

        pid_control(error, speed_);

    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 현재 속도 저장 (x축 속도)
        speed_ = msg->twist.twist.linear.x;
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
