#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"  // odom 메시지 추가
#include <string>
#include <vector>
#include <cmath>

class DisparityFollowGap : public rclcpp::Node {
public:
    DisparityFollowGap() : Node("disparity_node") {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DisparityFollowGap::lidar_callback, this, std::placeholders::_1));
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&DisparityFollowGap::odom_callback, this, std::placeholders::_1));
        
        current_steering_angle_ = 0.0;  // 초기화
        current_speed_ = 0.0;  // 초기화
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    double current_steering_angle_;  // 차량의 스티어링 각도
    double current_speed_;  // 차량의 속도

    // Lidar 데이터 전처리 (scan_threshold는 차량 속도에 따라 동적으로 설정)
    std::vector<float> preprocess_lidar(const std::vector<float>& ranges) {
        double scan_threshold = 1.5 + 0.5 * current_speed_;  // 속도에 따라 동적으로 scan_threshold 설정
        std::vector<float> filtered_ranges = ranges;
        
        for (auto& range : filtered_ranges) {
            if (range > scan_threshold) {
                range = scan_threshold;
            }
        }
        
        int window_size = 5;
        for (size_t i = window_size; i < filtered_ranges.size() - window_size; ++i) {
            float sum = 0.0;
            for (int j = -window_size; j <= window_size; ++j) {
                sum += filtered_ranges[i + j];
            }
            filtered_ranges[i] = sum / (2 * window_size + 1);
        }
        
        return filtered_ranges;
    }

    std::vector<int> find_discontinuities(const std::vector<float>& ranges, double base_threshold) {
        std::vector<int> discontinuities;
        
        for (size_t i = 0; i < ranges.size() - 1; i++) {
            double diff = std::abs(ranges[i] - ranges[i + 1]);
            double adaptive_threshold = base_threshold * std::log1p(ranges[i]);
            
            if (diff > adaptive_threshold) {
                discontinuities.push_back(i);
            }
        }
        
        return discontinuities;
    }

    void apply_bubbles(std::vector<float>& ranges, std::vector<int>& discontinuities, float car_radius, float angle_increment) {
        std::vector<int> updated_discontinuities;
        
        for (int index : discontinuities) {
            double diff = ranges[index] - ranges[index + 1];
            int car_bubble_point = static_cast<int>((car_radius / ranges[index]) / angle_increment);
    
            if (diff < 0) {  // 값이 증가 → 작은 값 기준으로 확장 (뒤쪽 확장)
                int new_end = std::min(index + car_bubble_point, static_cast<int>(ranges.size()) - 1);
                for (int j = index; j <= new_end; ++j) {
                    ranges[j] = ranges[index];  // 값 확장
                }
                updated_discontinuities.push_back(new_end);
            } 
            else if (diff > 0) {  // 값이 감소 → 작은 값 기준으로 확장 (앞쪽 확장)
                int new_start = std::max(0, index - car_bubble_point);
                for (int j = new_start; j <= index; ++j) {
                    ranges[j] = ranges[index + 1];  // 값 확장
                }
                updated_discontinuities.push_back(new_start);
            }
        }
    
        std::sort(updated_discontinuities.begin(), updated_discontinuities.end());
        discontinuities = updated_discontinuities;
    }
    
    std::pair<int, int> find_best_gap(const std::vector<float>& ranges, const std::vector<int>& discontinuities) {
        std::vector<std::pair<int, int>> gaps;
        int start = 0;
    
        // 갭을 생성
        for (int index : discontinuities) {
            gaps.emplace_back(start, index - 1);
            start = index + 1;
        }
        gaps.emplace_back(start, ranges.size() - 1);
    
        // 최대 값들 찾기
        double max_value = *std::max_element(ranges.begin(), ranges.end());
    
        // 최대 값의 인덱스를 모두 찾음
        std::vector<int> max_indices;
        for (int i = 0; i < ranges.size(); ++i) {
            if (ranges[i] == max_value) {
                max_indices.push_back(i);
            }
        }
    
        std::pair<int, int> best_gap = {0, 0};
        int max_gap_size = 0;
    
        // 모든 최대 값에 대해서, 그 값이 포함된 갭을 찾고 가장 넓은 갭을 선택
        for (const auto& gap : gaps) {
            for (int max_index : max_indices) {
                if (max_index >= gap.first && max_index <= gap.second) {
                    int gap_size = gap.second - gap.first;
                    if (gap_size > max_gap_size) {
                        max_gap_size = gap_size;
                        best_gap = gap;
                    }
                }
            }
        }
    
        return best_gap;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 차량의 스티어링 각도와 속도 업데이트
        current_steering_angle_ = msg->pose.pose.orientation.y;  // 예시로 z축 사용, 실제는 차량에 맞게 조정 필요
        current_speed_ = msg->twist.twist.linear.x;  // 차량의 속도
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        std::vector<float> filtered_ranges = preprocess_lidar(scan_msg->ranges);  // 속도 기반으로 scan_threshold 동적 설정
        std::vector<int> discontinuities = find_discontinuities(filtered_ranges, 0.1);  // threshold를 기본값으로 설정
        apply_bubbles(filtered_ranges, discontinuities, 0.4, scan_msg->angle_increment);  // 차량 반경 0.4m로 가정

        // 갭 찾기
        std::pair<int, int> best_gap = find_best_gap(filtered_ranges, discontinuities);
    
        // 갭 내의 가장 큰 값들 중에서 가장 가까운 값을 선택하기
        std::vector<int> best_gap_indices;
        double max_value = *std::max_element(filtered_ranges.begin() + best_gap.first, filtered_ranges.begin() + best_gap.second + 1);
        for (int i = best_gap.first; i <= best_gap.second; ++i) {
            if (filtered_ranges[i] == max_value) {
                best_gap_indices.push_back(i);
            }
        }
    
        // 차량의 현재 스티어링 각도 사용
        double current_angle = current_steering_angle_;  // odom에서 받아온 스티어링 각도 사용
        int closest_index = best_gap_indices[0];
        double min_diff = std::abs(current_angle - (scan_msg->angle_min + closest_index * scan_msg->angle_increment));
    
        for (int idx : best_gap_indices) {
            double diff = std::abs(current_angle - (scan_msg->angle_min + idx * scan_msg->angle_increment));
            if (diff < min_diff) {
                closest_index = idx;
                min_diff = diff;
            }
        }
    
        // 최종 스티어링 각도 계산
        double steering_angle = scan_msg->angle_min + closest_index * scan_msg->angle_increment;
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering_angle;

        double steering_degree = std::abs(steering_angle * 180 / M_PI);
        double drive_speed = 0.0;
        //-------------------- Normal Mode -----------------//
        if (steering_degree <= 5.0) {  // 거의 직진
            drive_speed = 1.2;
        } else if (steering_degree <= 10.0) {  // 약간의 커브
            drive_speed = 1.0;
        } else if (steering_degree <= 15.0) {  // 완만한 커브
            drive_speed = 0.8;
        } else {  // 중간 커브
            drive_speed = 0.5;
        }
        drive_msg.drive.speed = drive_speed;  // 차량의 실제 속도를 반영
        
        drive_pub_->publish(drive_msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisparityFollowGap>());
    rclcpp::shutdown();
    return 0;
}