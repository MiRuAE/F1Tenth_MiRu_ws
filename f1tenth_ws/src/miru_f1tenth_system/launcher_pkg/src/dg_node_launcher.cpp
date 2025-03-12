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
    NodeLauncher() : Node("dg_node_launcher") 
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&NodeLauncher::lidar_callback, this, std::placeholders::_1));
    }

private:
    std::string lidarscan_topic = "/scan";
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    double scan_threshold = 3.0;
    int left_min_index = -1;
    int right_min_index = -1;
    double wall_threshold = 1.5;
    int lidar_center = 540;
    int data_size = 0;

    std::vector<float> preprocess_lidar(std::vector<float>& ranges)
    {   
        if (ranges.empty()) {
            return ranges;
        }

        int window_size = 9;
        int padding = window_size / 2;
        data_size = static_cast<int>(ranges.size());

        std::vector<float> padded(data_size + padding * 2);

        std::fill(padded.begin(), padded.begin() + padding, ranges.front());
        std::copy(ranges.begin(), ranges.end(), padded.begin() + padding);
        std::fill(padded.begin() + padding + data_size, padded.end(), ranges.back());

        for(int i = 0; i < data_size + window_size; i++){
            if(padded[i] > scan_threshold)
                padded[i] = scan_threshold;
        }
        
        for(int i = 0; i < data_size; i++){
            float sum = 0.0;
            for(int j = 0; j < window_size; j++){
                sum += padded[i + j];
            }
            ranges[i] = sum / window_size;
        }
        return ranges;
    }

    bool is_left_wall_start(std::vector<float>& ranges) {
        left_min_index = std::min_element(ranges.begin() + lidar_center, ranges.end()) - ranges.begin();
        if (ranges[left_min_index] > wall_threshold) return false;
        
        std::vector<float> group = {ranges[left_min_index]};
        double tolerance = 0.2;
        int required_group_size = 300;

        for (int i = left_min_index - 1, j = left_min_index + 1; i >= lidar_center || j < ranges.size(); i--, j++) {
            float last_value = group.back();
            
            if (i >= lidar_center && std::abs(ranges[i] - last_value) < tolerance) {
                group.push_back(ranges[i]);
            }
            if (j < ranges.size() && std::abs(ranges[j] - last_value) < tolerance) {
                group.push_back(ranges[j]);
            }
            if (group.size() >= required_group_size) {
                RCLCPP_INFO(this->get_logger(), "Left Wall detected!");
                return true;
            }
        }
        return false;
    }
    
    bool is_right_wall_start(std::vector<float>& ranges) {
        right_min_index = std::min_element(ranges.begin(), ranges.begin() + lidar_center) - ranges.begin();
        if (ranges[right_min_index] > wall_threshold) return false;
        
        std::vector<float> group = {ranges[right_min_index]};
        double tolerance = 0.2;
        int required_group_size = 300;

        for (int i = right_min_index - 1, j = right_min_index + 1; i >= 0 || j < lidar_center; i--, j++) {
            float last_value = group.back();
            
            if (i >= 0 && std::abs(ranges[i] - last_value) < tolerance) {
                group.push_back(ranges[i]);
            }
            if (j < lidar_center && std::abs(ranges[j] - last_value) < tolerance) {
                group.push_back(ranges[j]);
            }
            if (group.size() >= required_group_size) {
                RCLCPP_INFO(this->get_logger(), "Right Wall detected!");
                return true;
            }
        }
        return false;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) 
    {
        if (scan_msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty scan message received");
            return;
        }

        std::vector<float> processed_ranges(scan_msg->ranges.begin(), scan_msg->ranges.end());
        processed_ranges = preprocess_lidar(processed_ranges);
        
        bool left_wall_detected = is_left_wall_start(processed_ranges);
        bool right_wall_detected = is_right_wall_start(processed_ranges);
        
        if (left_wall_detected && right_wall_detected) {
            double angle_threshold = 90.0 * (M_PI / 180.0);
            double angle_difference = std::abs(left_min_index - right_min_index) * scan_msg->angle_increment;
            
            if (angle_difference > angle_threshold) {
                RCLCPP_INFO(this->get_logger(), "B sector detected - both walls with sufficient separation");
            } else {
                RCLCPP_INFO(this->get_logger(), "Both walls detected but insufficient separation");
            }
        }
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
