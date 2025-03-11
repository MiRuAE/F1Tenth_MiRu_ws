#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class NodeLauncher : public rclcpp::Node {
public:
    NodeLauncher() : Node("sh_node_launcher") 
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, 
            std::bind(&NodeLauncher::lidar_callback, this, std::placeholders::_1));
        
        // Define the distance thresholds
        threshold_ = this->declare_parameter("threshold", 1.0);
        current_mode = this->declare_parameter("current_mode", 1);
    }

private:
    std::string lidarscan_topic = "/scan";
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    double threshold_;
    int current_mode;
    
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) 
    {

        //Mode A
        if (current_mode == 1){

            std::vector<int> disparity_indices;
            float disparity_threshold = 0.05;
            float max_angle = 0.0;

            //Disparity index sense
            std::vector<std::pair<int, int>> disparity_pairs;  
            int low_index = -1;
            bool is_low_detected = false;

            for (size_t i = 1; i < scan_msg->ranges.size(); ++i) {
                float diff = scan_msg->ranges[i] - scan_msg->ranges[i - 1];

                if (std::isfinite(diff)) {
                    if (diff <= -disparity_threshold) {
                        low_index = i;  
                        is_low_detected = true;
                    } 
                    else if (diff >= disparity_threshold && is_low_detected) {
                        disparity_pairs.emplace_back(low_index, i);
                        is_low_detected = false;
                    }
                }
            }

            //maximun angle calculate
            if (disparity_pairs.empty()) {
                RCLCPP_INFO(this->get_logger(), "No disparity found");
            }
            else {
                for (const auto& pair : disparity_pairs) {
                    if (static_cast<float>(pair.second - pair.first) / 4 > max_angle) {
                        max_angle = static_cast<float>(pair.second - pair.first) / 4;
                    }
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Maximun angle : %0.3f", max_angle);

            if (max_angle >= 180.0){
                current_mode = 2;
                this->set_parameter(rclcpp::Parameter("current_mode", rclcpp::ParameterValue(current_mode)));
            }
            
        }

        RCLCPP_INFO(this->get_logger(), "Current Mode : %d \n", current_mode);
        // //Mode B
        // else if (current_mode == 2){

        //     std::vector<int> disparity_indices;
        //     float disparity_threshold = 1.0;  
            
        //     for (size_t i = 1; i < scan_msg->ranges.size(); ++i) {
        //         float diff = std::abs(scan_msg->ranges[i] - scan_msg->ranges[i - 1]);  
        //         if (std::isfinite(diff) && diff >= disparity_threshold) {
        //             disparity_indices.push_back(i); 
        //         }
        //     }
            

        //     if (true){
        //         current_mode_ = 3;
        //         this->set_parameter(rclcpp::Parameter("current_mode", current_mode_));
        //     }
        // }
        
        
        
        
        
        
        
        // // Log the information
        // RCLCPP_INFO(this->get_logger(), 
        //             "Scan summary: %zu total points, %zu valid points", 
        //             scan_msg->ranges.size(), valid_ranges.size());
        
        // RCLCPP_INFO(this->get_logger(), 
        //             "Mean range: %.2f meters", mean_range);
        
        // RCLCPP_INFO(this->get_logger(), 
        //             "Distance breakdown: Close (<%.1fm): %d (%.1f%%), Medium (%.1f-%.1fm): %d (%.1f%%), Far (>%.1fm): %d (%.1f%%)",
        //             close_threshold_, close_count, close_percent,
        //             close_threshold_, medium_threshold_, medium_count, medium_percent,
        //             medium_threshold_, far_count, far_percent);
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
