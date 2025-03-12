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
    NodeLauncher() : Node("mk_node_launcher") 
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&NodeLauncher::lidar_callback, this, std::placeholders::_1));
        
        //scan_threshold_ = this->declare_parameter("scan_threshold", 3.0);                                                                         
    }

private:
    std::string lidarscan_topic = "/scan";
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    double scan_threshold = 3.0;
    int left_min_index = -1;
    int right_min_index = -1;
    double wall_threshold = 1.0;
    int lidar_center = 540;
    int data_size = 0; // Added data_size declaration

    std::vector<float> preprocess_lidar(std::vector<float>& ranges)
    {   
        if (ranges.empty()) {
            return ranges;
        }

        // Preprocess the LiDAR scan array. Expert implementation includes:
        // Cuttoff with threshold
        // double scan_threshold = 2.5;
        // double min_range = *std::min_element(ranges.begin(), ranges.end());
        // RCLCPP_INFO(this->get_logger(), "Min_range: %f", min_range);

        int window_size = 9;
        int padding = window_size / 2;
        data_size = static_cast<int>(ranges.size());

        std::vector<float> padded(data_size + padding * 2);

        // left padding
        std::fill(padded.begin(), padded.begin() + padding, ranges.front());

        // copy origin
        std::copy(ranges.begin(), ranges.end(), padded.begin() + padding);

        // right padding
        std::fill(padded.begin() + padding + data_size, padded.end(), ranges.back());

        // 1.Rejecting high values (eg. > 3m)
        for(int i = 0; i < data_size + window_size; i++){
            if(padded[i] > scan_threshold)
            padded[i] = scan_threshold;
        }
        
        // 2.Setting each value to the mean over some window
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
        //RCLCPP_INFO(this->get_logger(), "Left min index: %d, min range: %f", left_min_index, ranges[left_min_index]);
        
        if (ranges[left_min_index] > wall_threshold) return false;
        
        std::vector<float> group = {ranges[left_min_index]};
        double tolerance = 0.1;
        int required_group_size = 200;
        
        for (int i = left_min_index - 1; i >= lidar_center; i--) {
            if (std::abs(ranges[i] - ranges[left_min_index]) < tolerance) {
                group.push_back(ranges[i]);
            } else {
                break;
            }
        }
        
        for (int j = left_min_index + 1; j < ranges.size(); j++) {
            if (std::abs(ranges[j] - ranges[left_min_index]) < tolerance) {
                group.push_back(ranges[j]);
            } else {
                break;
            }
        }
        int group_point_num = group.size();
        if (group_point_num >= required_group_size) {
            RCLCPP_INFO(this->get_logger(), "Left Wall detected! Group size: %d", group_point_num);
            return true;
        }
        return false;
    }
    
    bool is_right_wall_start(std::vector<float>& ranges) {
        right_min_index = std::min_element(ranges.begin(), ranges.begin() + lidar_center) - ranges.begin();
        //RCLCPP_INFO(this->get_logger(), "Right min index: %d, min range: %f", right_min_index, ranges[right_min_index]);
        
        if (ranges[right_min_index] > wall_threshold) return false;
        
        std::vector<float> group = {ranges[right_min_index]};
        double tolerance = 0.1;
        int required_group_size = 200;
        
        for (int i = right_min_index - 1; i >= 0; i--) {
            if (std::abs(ranges[i] - ranges[right_min_index]) < tolerance) {
                group.push_back(ranges[i]);
            } else {
                break;
            }
        }
        
        for (int j = right_min_index + 1; j < lidar_center; j++) {
            if (std::abs(ranges[j] - ranges[right_min_index]) < tolerance) {
                group.push_back(ranges[j]);
            } else {
                break;
            }
        }
        int group_point_num = group.size();
        if (group_point_num >= required_group_size) {
            RCLCPP_INFO(this->get_logger(), "Right Wall detected! Group size: %d", group_point_num);
            return true;
        }
        return false;
    }

    bool detect_corridor(std::vector<float>& ranges, double angle_increment) {
        double angle_threshold = 180.0 * (M_PI / 180.0); // 180도 이상 차이
        
        if (is_left_wall_start(ranges) && is_right_wall_start(ranges)) {
            double angle_difference = std::abs(left_min_index - right_min_index) * angle_increment;
            
            if (angle_difference > angle_threshold) {
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
        
        // 개별적으로 각 벽 감지 함수 호출
        bool left_wall_detected = is_left_wall_start(processed_ranges);
        bool right_wall_detected = is_right_wall_start(processed_ranges);
        
        // 개별 벽 감지 결과 출력 (함수 내부에서 이미 로그를 출력하고 있으므로 생략 가능)
        // if (left_wall_detected) {
        //     RCLCPP_INFO(this->get_logger(), "Left Wall detected!");
        // }
        // if (right_wall_detected) {
        //     RCLCPP_INFO(this->get_logger(), "Right Wall detected!");
        // }
        
        // 복도 감지 로직
        if (left_wall_detected && right_wall_detected) {
            double angle_threshold = 90.0 * (M_PI / 180.0); // 90도 이상 차이
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
