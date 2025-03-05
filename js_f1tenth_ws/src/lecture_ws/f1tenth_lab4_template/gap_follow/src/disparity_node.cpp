#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

// Structure to track gaps (continuous free space)
struct Gap {
    int start_idx;
    int end_idx;
    float width_angle;  // Width in radians
    float max_depth;    // Maximum depth in this gap
    int best_idx;       // Index of best point (typically deepest point)
};

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car with disparity extender concepts

public:
    ReactiveFollowGap() : Node("reactive_follow_gap")
    {
        // Declare parameters
        this->declare_parameter("car_width", 0.4);
        this->declare_parameter("disparity_threshold", 0.3);
        this->declare_parameter("min_speed", 0.5);
        this->declare_parameter("max_speed", 3.5);
        this->declare_parameter("absolute_max_speed", 6.0);
        this->declare_parameter("min_distance", 0.5);
        this->declare_parameter("max_distance", 3.0);
        this->declare_parameter("absolute_max_distance", 6.0);
        this->declare_parameter("coefficient_of_friction", 0.62);
        this->declare_parameter("wheelbase_width", 0.328);
        this->declare_parameter("left_wing", M_PI / 2.7);
        this->declare_parameter("right_wing", -M_PI / 2.7);
        this->declare_parameter("obstacle_threshold", 1.0);
        this->declare_parameter("min_gap_width", 0.2);
        this->declare_parameter("heading_weight", 0.5);
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("drive_topic", "/drive");
        
        // Get parameters
        car_width = this->get_parameter("car_width").as_double();
        disparity_threshold = this->get_parameter("disparity_threshold").as_double();
        min_speed = this->get_parameter("min_speed").as_double();
        max_speed = this->get_parameter("max_speed").as_double();
        absolute_max_speed = this->get_parameter("absolute_max_speed").as_double();
        min_distance = this->get_parameter("min_distance").as_double();
        max_distance = this->get_parameter("max_distance").as_double();
        absolute_max_distance = this->get_parameter("absolute_max_distance").as_double();
        coefficient_of_friction = this->get_parameter("coefficient_of_friction").as_double();
        wheelbase_width = this->get_parameter("wheelbase_width").as_double();
        left_wing = this->get_parameter("left_wing").as_double();
        right_wing = this->get_parameter("right_wing").as_double();
        obstacle_threshold = this->get_parameter("obstacle_threshold").as_double();
        min_gap_width = this->get_parameter("min_gap_width").as_double();
        heading_weight = this->get_parameter("heading_weight").as_double();
        scan_topic = this->get_parameter("scan_topic").as_string();
        drive_topic = this->get_parameter("drive_topic").as_string();

        // Create subscribers and publishers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), "ReactiveFollowGap node initialized");
    }

private:
    // ROS communication
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    
    // Configuration parameters
    std::string scan_topic;
    std::string drive_topic;
    double car_width;
    double disparity_threshold;
    double min_speed;
    double max_speed;
    double absolute_max_speed;
    double min_distance;
    double max_distance;
    double absolute_max_distance;
    double coefficient_of_friction;
    double wheelbase_width;
    double left_wing;
    double right_wing;
    double obstacle_threshold;
    double min_gap_width;
    double heading_weight;
    
    // State variables
    int data_size = 0;
    int left_wing_index = 0;
    int right_wing_index = 0;
    int prev_best_idx = -1;
    const double gravity = 9.81998;
    
    // Find disparities (sudden changes in distance) in the range data
    std::vector<int> find_disparities(const std::vector<float>& ranges, float threshold) {
        std::vector<int> disparity_indices;
        
        // Scan for disparities (only in forward region)
        for (int i = right_wing_index; i < left_wing_index; i++) {
            if (std::abs(ranges[i] - ranges[i + 1]) >= threshold) {
                disparity_indices.push_back(i);
            }
        }
        
        return disparity_indices;
    }

    // Calculate number of samples to extend disparity based on car width
    int calculate_samples_for_width(float distance, float car_width, float angle_increment) {
        // Calculate arc length
        float arc_length = angle_increment * distance;
        
        // Number of samples needed to cover car_width
        return std::ceil(car_width / arc_length);
    }

    // Extend disparities to account for vehicle width
    std::vector<float> extend_disparities(const std::vector<float>& ranges, 
                                        const std::vector<int>& disparity_indices, 
                                        float car_width, float angle_increment) {
        std::vector<float> processed_ranges = ranges;
        
        for (int idx : disparity_indices) {
            // Get values on both sides of disparity
            float value1 = ranges[idx];
            float value2 = ranges[idx + 1];
            
            // Determine which side is closer (has smaller range)
            bool extend_positive;
            float nearer_value;
            int nearer_index;
            
            if (value1 < value2) {
                // Left point is closer
                nearer_value = value1;
                nearer_index = idx;
                extend_positive = true;  // Extend to the right
            } else {
                // Right point is closer
                nearer_value = value2;
                nearer_index = idx + 1;
                extend_positive = false;  // Extend to the left
            }
            
            // Calculate how many samples to extend
            int samples_to_extend = calculate_samples_for_width(nearer_value, car_width, angle_increment);
            
            // Extend the disparity
            int current_index = nearer_index;
            for (int i = 0; i < samples_to_extend; i++) {
                // Make sure we stay within bounds
                if (current_index < right_wing_index) {
                    current_index = right_wing_index;
                    break;
                }
                if (current_index > left_wing_index) {
                    current_index = left_wing_index;
                    break;
                }
                
                // Only update if current value is farther than nearer_value
                if (processed_ranges[current_index] > nearer_value) {
                    processed_ranges[current_index] = nearer_value;
                }
                
                // Move in appropriate direction
                if (extend_positive) {
                    current_index++;
                } else {
                    current_index--;
                }
            }
        }
        
        return processed_ranges;
    }

    // Apply window averaging to smooth the range data
    std::vector<float> window_average(const std::vector<float>& ranges, int window_size) {
        std::vector<float> smoothed = ranges;
        int half_window = window_size / 2;
        
        for (int i = right_wing_index + half_window; i <= left_wing_index - half_window; i++) {
            float sum = 0.0;
            for (int j = -half_window; j <= half_window; j++) {
                sum += ranges[i + j];
            }
            smoothed[i] = sum / window_size;
        }
        
        return smoothed;
    }

    // Find all gaps in the processed ranges
    std::vector<Gap> find_gaps(const std::vector<float>& ranges, float angle_increment, 
                              float min_gap_width, float obstacle_threshold) {
        std::vector<Gap> gaps;
        bool in_gap = false;
        Gap current_gap;
        current_gap.max_depth = 0.0;
        
        // Scan through ranges to find gaps
        for (int i = right_wing_index; i <= left_wing_index; i++) {
            // If range is greater than threshold, we're in free space
            if (ranges[i] > obstacle_threshold) {
                // Start of a new gap
                if (!in_gap) {
                    current_gap.start_idx = i;
                    current_gap.max_depth = ranges[i];
                    current_gap.best_idx = i;
                    in_gap = true;
                }
                // Continue existing gap, update max depth if needed
                else if (ranges[i] > current_gap.max_depth) {
                    current_gap.max_depth = ranges[i];
                    current_gap.best_idx = i;
                }
            }
            // We've found an obstacle, end of current gap
            else if (in_gap) {
                current_gap.end_idx = i - 1;
                current_gap.width_angle = (current_gap.end_idx - current_gap.start_idx) * angle_increment;
                
                // Only add gap if it's wide enough
                if (current_gap.width_angle >= min_gap_width) {
                    gaps.push_back(current_gap);
                }
                
                in_gap = false;
                current_gap.max_depth = 0.0;
            }
        }
        
        // Don't forget to close the last gap if we ended in one
        if (in_gap) {
            current_gap.end_idx = left_wing_index;
            current_gap.width_angle = (current_gap.end_idx - current_gap.start_idx) * angle_increment;
            
            if (current_gap.width_angle >= min_gap_width) {
                gaps.push_back(current_gap);
            }
        }
        
        return gaps;
    }

    // Select best gap based on scoring
    int select_best_gap_point(const std::vector<Gap>& gaps, int prev_best_idx, float heading_weight) {
        if (gaps.empty()) {
            return (right_wing_index + left_wing_index) / 2; // Default to middle if no gaps
        }
        
        // Find largest gap (by depth * width)
        float best_score = 0;
        int best_gap_idx = 0;
        
        for (size_t i = 0; i < gaps.size(); i++) {
            // Score based on depth and width
            float depth_score = gaps[i].max_depth;
            float width_score = gaps[i].width_angle;
            
            // Prefer gaps in current heading direction
            float heading_score = 1.0;
            if (prev_best_idx > 0) {
                float heading_diff = std::abs(gaps[i].best_idx - prev_best_idx);
                heading_score = std::exp(-heading_diff * 0.01); // Exponential falloff
            }
            
            float score = depth_score * width_score * (1.0 + heading_weight * heading_score);
            
            if (score > best_score) {
                best_score = score;
                best_gap_idx = i;
            }
        }
        
        // Use the deepest point in the best gap
        return gaps[best_gap_idx].best_idx;
    }

    // Calculate optimal speed based on turning radius and forward distance
    double calculate_speed(double steering_angle, double forward_distance) {
        // Calculate turning radius based on steering angle
        double angle_abs = std::abs(steering_angle);
        double speed;
        
        if (angle_abs < 0.0872665) {  // Less than 5 degrees
            speed = max_speed;
        } else {
            double turning_radius = wheelbase_width / std::sin(angle_abs);
            double physics_max_velocity = std::sqrt(coefficient_of_friction * gravity * turning_radius);
            
            // Scale based on how close to max_speed
            if (physics_max_velocity < max_speed) {
                speed = physics_max_velocity * (physics_max_velocity / max_speed);
            } else {
                speed = max_speed;
            }
        }
        
        // Scale speed based on forward distance
        if (forward_distance > absolute_max_distance) {
            speed = absolute_max_speed;
        } else if (forward_distance > max_distance) {
            // Linearly scale between max_speed and absolute_max_speed
            double scale_factor = (forward_distance - max_distance) / 
                                 (absolute_max_distance - max_distance);
            speed = max_speed + scale_factor * (absolute_max_speed - max_speed);
        } else if (forward_distance < min_distance) {
            // Too close, slow down or reverse
            speed = (forward_distance < 0.25) ? -0.5 : min_speed;
        } else {
            // Scale linearly between min_speed and calculated speed
            double scale_factor = (forward_distance - min_distance) / 
                                 (max_distance - min_distance);
            speed = min_speed + scale_factor * (speed - min_speed);
        }
        
        return speed;
    }

    // Enhanced preprocessing pipeline
    std::vector<float> process_scan(const std::vector<float>& ranges, float angle_increment) {
        // Step 1: Find disparities
        std::vector<int> disparities = find_disparities(ranges, disparity_threshold);
        
        // Step 2: Extend disparities to account for car width
        std::vector<float> extended_ranges = extend_disparities(
            ranges, disparities, car_width, angle_increment);
        
        // Step 3: Apply window averaging to smooth data 
        std::vector<float> smoothed_ranges = window_average(extended_ranges, 5);
        
        return smoothed_ranges;
    }

    // Main callback for LIDAR data
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        if (scan_msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty scan message received");
            return;
        }
        
        // Get scan data
        data_size = static_cast<int>(scan_msg->ranges.size());
        left_wing_index = static_cast<int>((left_wing - scan_msg->angle_min) / scan_msg->angle_increment);
        right_wing_index = static_cast<int>((right_wing - scan_msg->angle_min) / scan_msg->angle_increment);
        
        // Bounds checking
        left_wing_index = std::min(left_wing_index, data_size - 1);
        right_wing_index = std::max(right_wing_index, 0);
        
        // Safely calculate gap mean
        double gap_stack = 0.0;
        for (int i = 0; i < data_size - 1; i++) {
            gap_stack += std::abs(scan_msg->ranges[i] - scan_msg->ranges[i + 1]);
        }
        double gap_mean = gap_stack / (data_size - 1);
        
        // Create a copy of the ranges
        std::vector<float> ranges(scan_msg->ranges.begin(), scan_msg->ranges.end());
        
        // Process scan data (disparity extension and smoothing)
        std::vector<float> processed_ranges = process_scan(ranges, scan_msg->angle_increment);
        
        // Find gaps in processed data
        std::vector<Gap> gaps = find_gaps(processed_ranges, scan_msg->angle_increment, 
                                         min_gap_width, obstacle_threshold);
        
        // Select best point
        int best_idx = select_best_gap_point(gaps, prev_best_idx, heading_weight);
        prev_best_idx = best_idx;  // Store for next iteration
        
        // Calculate steering angle and appropriate speed
        double steering_angle = scan_msg->angle_min + (scan_msg->angle_increment * best_idx);
        
        // Forward distance (straight ahead)
        int center_idx = (left_wing_index + right_wing_index) / 2;
        double forward_distance = processed_ranges[center_idx];
        
        // Calculate physics-based speed
        double drive_speed = calculate_speed(steering_angle, forward_distance);
        
        // Publish drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;
        
        drive_pub_->publish(drive_msg);
        
        // Log debug information
        RCLCPP_DEBUG(this->get_logger(), 
                    "Steering: %.2f, Speed: %.2f, Forward dist: %.2f, Gaps found: %zu", 
                    steering_angle, drive_speed, forward_distance, gaps.size());
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
