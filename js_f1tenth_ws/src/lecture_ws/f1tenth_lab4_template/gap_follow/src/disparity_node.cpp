#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#inclue <vector>
#include <cmath>
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));

        // AckermannDriveStamped 메시지를 발행하는 퍼블리셔 생성
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    int data_size = 0;
    double car_width = 0.4;
    double left_wing = M_PI / 2.7;
    double right_wing = -(M_PI / 2.7);
    int left_wing_index = 0;
    int right_wing_index = 0;

    std::vector<float> preprocess_lidar(std::vector<float>& ranges, double gap_mean, double angle_increment)
    {   
        if (ranges.empty()) {
            return ranges;
        }
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        double scan_threshold = 3.0;
        int window_size = 9;
        int padding = window_size / 2;
        dobule car_radius = car_width / 2;
        int car_bubble_point = 0;

        // From right_wing_index ~ left_wing_index, scan for the disparity spots
        for(i = right_wing_index; i <= left_wing_index; i ++) {
            double diff = ranges[i] - ranges[i+1];
            if(diff < 0 && -diff > gap_mean)
            {
                car_bubble_point = (car_radius / ranges[i]) / angle_increment;
                for(j = i - car_bubble_point; j <= i; j ++) {
                    ranges[j] = ranges[i];
                }
            }
            else if(diff > 0 && diff > gap_mean)
            {
                car_bubble_point = (car_radius / ranges[i+1]) / angle_increment;
                for(j = i; j <= i + car_bubble_point; j ++) {
                    ranges[j] = ranges[i+1];
                }
            }
        }

        // 2.Setting each value to the mean over some window
        for(int i = right_wing_index; i < left_wing_index; i++){
          float sum = 0.0;
          for(int j = 0; j < window_size; j++){
            sum += ranges[i + j];
          }
          ranges[i] = sum / window_size;
          
        }

        return ranges;
    }

    

    void find_max_gap(float* ranges, int* indice)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        return;
    }

    void find_best_point(float* ranges, int* indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        return;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        if (scan_msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty scan message received");
            return;
        }

        data_size = static_cast<int>(scan_msg->ranges.size());

        left_wing_index = (left_wing - scan_msg->angle_min) / scan_msg->angle_increment;
        right_wing_index = (right_wing - scan_msg->angle_min) / scan_msg->angle_increment;

        double gap_stack = 0.0;
        for(int i = 0; i < data_size; i++){
            double gap = std::abs(scan_msg->ranges[i] - scan_msg->ranges[i+1]);
            gap_stack += gap;
        }
        double gap_mean = gap_stack / data_size;
        double angle_increment = scan_msg->angle_increment;

        std::vector<float> processed_ranges(scan_msg->ranges.begin(), scan_msg->ranges.end());
        processed_ranges = preprocess_lidar(scan_msg->ranges, gap_mean, angle_increment);
        double min_range = 100.0;
        int min_index = 0;

        
        /// TODO:
        // Find disparity point to LiDAR
        

        // Eliminate all points inside 'bubble' (set them to zero) 

        // Find max length gap 

        // Find the best point in the gap 

        // Publish Drive message
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
