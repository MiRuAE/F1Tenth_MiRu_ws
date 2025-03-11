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
    double wall_threshold = 1.5;
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
        // double wall_threshold = 1.5; // 벽으로 판단할 최대 거리
        // int lidar_center = 540; // LiDAR 중앙 인덱스
        
        // 왼쪽 절반에서 최소 거리 찾기
        left_min_index = std::min_element(ranges.begin(), ranges.begin() + lidar_center) - ranges.begin();
        
        if (ranges[left_min_index] > wall_threshold) return false;
        
        std::vector<float> group = {ranges[left_min_index]}; // Fixed: min_index -> left_min_index
        double tolerance = 0.2; // 주변 값과 비교할 허용 오차
        int required_group_size = 100; // 벽으로 판단하기 위한 최소 연속 그룹 개수
        
        // 그룹의 중앙값 계산 함수
        auto get_median = [](std::vector<float>& group) -> float {
            std::vector<float> temp = group;
            std::sort(temp.begin(), temp.end());
            return temp[temp.size() / 2];
        };
        
        // 양쪽 방향으로 연속된 유사한 값 찾기
        for (int i = left_min_index - 1, j = left_min_index + 1; i >= 0 || j < lidar_center; i--, j++) {
            float median = get_median(group);
            
            if (i >= 0 && std::abs(ranges[i] - median) < tolerance) {
                group.push_back(ranges[i]);
            }
            if (j < lidar_center && std::abs(ranges[j] - median) < tolerance) {
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
        // double wall_threshold = 1.5; // 벽으로 판단할 최대 거리
        // int lidar_center = 540; // LiDAR 중앙 인덱스
        
        // 오른쪽 절반에서 최소 거리 찾기
        right_min_index = std::min_element(ranges.begin() + lidar_center, ranges.end()) - ranges.begin();
        
        if (ranges[right_min_index] > wall_threshold) return false; // Fixed: min_index -> right_min_index
        
        std::vector<float> group = {ranges[right_min_index]}; // Fixed: min_index -> right_min_index
        double tolerance = 0.2; // 주변 값과 비교할 허용 오차
        int required_group_size = 300; // 벽으로 판단하기 위한 최소 연속 그룹 개수
        
        // 그룹의 중앙값 계산 함수
        auto get_median = [](std::vector<float>& group) -> float {
            std::vector<float> temp = group;
            std::sort(temp.begin(), temp.end());
            return temp[temp.size() / 2];
        };
        
        // 양쪽 방향으로 연속된 유사한 값 찾기
        for (int i = right_min_index - 1, j = right_min_index + 1; i >= lidar_center || j < ranges.size(); i--, j++) {
            float median = get_median(group);
            
            if (i >= lidar_center && std::abs(ranges[i] - median) < tolerance) {
                group.push_back(ranges[i]);
            }
            if (j < ranges.size() && std::abs(ranges[j] - median) < tolerance) {
                group.push_back(ranges[j]);
            }
            if (group.size() >= required_group_size) {
                RCLCPP_INFO(this->get_logger(), "Right Wall detected!");
                return true;
            }
        }
        return false;
    }

    bool detect_corridor(std::vector<float>& ranges, double angle_increment) {
        double angle_threshold = 90.0 * (M_PI / 180.0); // 90도 이상 차이
        
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

        // Fixed: proper vector initialization
        std::vector<float> processed_ranges(scan_msg->ranges.begin(), scan_msg->ranges.end());
        processed_ranges = preprocess_lidar(processed_ranges); // Fixed: pass the vector, not scan_msg->ranges()
        
        if(detect_corridor(processed_ranges, scan_msg->angle_increment)){
            RCLCPP_INFO(this->get_logger(), "B sector detected");
        }

        // Rest of the code...
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
