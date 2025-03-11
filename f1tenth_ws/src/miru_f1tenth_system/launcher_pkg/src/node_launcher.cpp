#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class NodeLauncher : public rclcpp::Node {
public:
    NodeLauncher() : Node("node_launcher") 
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, 
            std::bind(&NodeLauncher::lidar_callback, this, std::placeholders::_1));
        
        // Define the distance thresholds (in meters)
        close_threshold_ = this->declare_parameter("close_threshold", 1.0);      // < 1.0m is close
        medium_threshold_ = this->declare_parameter("medium_threshold", 3.0);    // < 3.0m is medium
                                                                                // >= 3.0m is far
    }

private:
    std::string lidarscan_topic = "/scan";
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    double close_threshold_;
    double medium_threshold_;
    
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) 
    {
        if (scan_msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty scan message received");
            return;
        }
        
        // Filter out invalid measurements (typically represented as inf or zeros)
        std::vector<float> valid_ranges;
        for (const auto& range : scan_msg->ranges) {
            if (std::isfinite(range) && range > 0.0) {
                valid_ranges.push_back(range);
            }
        }
        
        if (valid_ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "No valid range measurements in scan");
            return;
        }
        
        // Calculate the mean of all valid ranges
        float mean_range = std::accumulate(valid_ranges.begin(), valid_ranges.end(), 0.0f) / valid_ranges.size();
        
        // Count ranges in each category
        int close_count = 0;
        int medium_count = 0;
        int far_count = 0;
        
        for (const auto& range : valid_ranges) {
            if (range < close_threshold_) {
                close_count++;
            } else if (range < medium_threshold_) {
                medium_count++;
            } else {
                far_count++;
            }
        }
        
        // Calculate percentages
        float close_percent = (static_cast<float>(close_count) / valid_ranges.size()) * 100.0f;
        float medium_percent = (static_cast<float>(medium_count) / valid_ranges.size()) * 100.0f;
        float far_percent = (static_cast<float>(far_count) / valid_ranges.size()) * 100.0f;
        
        // Log the information
        RCLCPP_INFO(this->get_logger(), 
                    "Scan summary: %zu total points, %zu valid points", 
                    scan_msg->ranges.size(), valid_ranges.size());
        
        RCLCPP_INFO(this->get_logger(), 
                    "Mean range: %.2f meters", mean_range);
        
        RCLCPP_INFO(this->get_logger(), 
                    "Distance breakdown: Close (<%.1fm): %d (%.1f%%), Medium (%.1f-%.1fm): %d (%.1f%%), Far (>%.1fm): %d (%.1f%%)",
                    close_threshold_, close_count, close_percent,
                    close_threshold_, medium_threshold_, medium_count, medium_percent,
                    medium_threshold_, far_count, far_percent);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<NodeLauncher>();
    RCLCPP_INFO(node->get_logger(), "Launching NodeLauncher...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}

  
  // 시스템 명령어 실행
  /*int result = std::system("ros2 run gap_follow reactive_node");*/
  /**/
  /*if (result == 0) {*/
  /*  printf("Successfully launched disparity_node\n");*/
  /*} else {*/
  /*  printf("Failed to launch disparity_node. Error code: %d\n", result);*/
  /*}*/
  
/*  return 0;*/
/*}*/
