#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "odom_msgs/msg/my_odom.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/string.hpp"

enum MissionState {
    MISSION_A,  // Camera mission
    MISSION_B,  // Lidar mission (in narrow walls)
    MISSION_C   // Odometry mission
};

class NodeLauncher : public rclcpp::Node {
public:
    NodeLauncher() : Node("js_node_launcher"), current_mission_(MISSION_A), 
                     consecutive_b_detections_(0), consecutive_c_detections_(0)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, 
            std::bind(&NodeLauncher::lidar_callback, this, std::placeholders::_1));
        
        // Subscribe to odometry for tracking position
        odom_sub_ = this->create_subscription<odom_msgs::msg::MyOdom>(
            "/my_odom", 10,
            std::bind(&NodeLauncher::odom_callback, this, std::placeholders::_1));

        // Define the distance thresholds (in meters)
        close_threshold_ = this->declare_parameter("close_threshold", 0.5);      // < 0.5m is close
        medium_threshold_ = this->declare_parameter("medium_threshold", 0.7);    // < 0.7m is medium
                                                                                // >= 0.7m is far

        // Define the threshold for mission transitions
        b_detection_threshold_ = this->declare_parameter("b_detection_threshold", 5);
        c_detection_threshold_ = this->declare_parameter("c_detection_threshold", 10);
        
        // Define target offset parameters for Mission C
        target_offset_x_ = this->declare_parameter("target_offset_x", 0.8);  // Forward offset from end of Mission B
        target_offset_y_ = this->declare_parameter("target_offset_y", 2.5);  // Right offset from end of Mission B
        
        // Publisher for mission state changes
        mission_publisher_ = this->create_publisher<std_msgs::msg::String>("current_mission", 10);
        
        // Publisher for transition position using custom odometry message
        transition_pub_ = this->create_publisher<odom_msgs::msg::MyOdom>("mission_b_end_position", 10);
        
        // Parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&NodeLauncher::onParameterChange, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Starting in Mission A (Camera)");
        RCLCPP_INFO(this->get_logger(), "Target offsets set to: forward=%.2f, right=%.2f", 
                    target_offset_x_, target_offset_y_);
        publish_mission_state("MISSION_A");
    }

private:
    std::string lidarscan_topic = "/scan";
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_publisher_;
    rclcpp::Subscription<odom_msgs::msg::MyOdom>::SharedPtr odom_sub_;
    rclcpp::Publisher<odom_msgs::msg::MyOdom>::SharedPtr transition_pub_;
    
    double close_threshold_;
    double medium_threshold_;
    MissionState current_mission_;
    int consecutive_b_detections_;
    int consecutive_c_detections_;
    int b_detection_threshold_;
    int c_detection_threshold_;
    
    // Store current position using custom odometry message
    odom_msgs::msg::MyOdom current_odom_;
    
    double target_offset_x_;
    double target_offset_y_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    void odom_callback(const odom_msgs::msg::MyOdom::SharedPtr msg) {
        current_odom_ = *msg;  // Store the entire message
    }

    void publish_mission_state(const std::string& mission_name) {
        auto message = std_msgs::msg::String();
        message.data = mission_name;
        mission_publisher_->publish(message);

        // If transitioning from B to C, publish the transition position with offsets
        if (mission_name == "MISSION_C" && current_mission_ == MISSION_B) {
            auto transition_msg = current_odom_;  // Use the stored custom odometry message
            transition_msg.header.stamp = this->now();
            
            // Calculate target position using current offsets
            double yaw_rad = current_odom_.yaw * M_PI / 180.0;  // Convert yaw to radians
            double global_offset_x = target_offset_x_ * std::cos(yaw_rad) - target_offset_y_ * std::sin(yaw_rad);
            double global_offset_y = target_offset_x_ * std::sin(yaw_rad) + target_offset_y_ * std::cos(yaw_rad);
            
            transition_msg.x = current_odom_.x + global_offset_x;
            transition_msg.y = current_odom_.y + global_offset_y;
            
            transition_pub_->publish(transition_msg);
            RCLCPP_INFO(this->get_logger(), 
                "Publishing transition: current (%.2f, %.2f, yaw: %.2f) with offset (%.2f, %.2f) -> target (%.2f, %.2f)",
                current_odom_.x, current_odom_.y, current_odom_.yaw,
                target_offset_x_, target_offset_y_,
                transition_msg.x, transition_msg.y);
        }
    }
    
    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : parameters) {
            if (param.get_name() == "target_offset_x") {
                target_offset_x_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated forward offset to: %.2f", target_offset_x_);
            }
            else if (param.get_name() == "target_offset_y") {
                target_offset_y_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated right offset to: %.2f", target_offset_y_);
            }
        }
        return result;
    }
    
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

        // Mission state transition logic
        switch (current_mission_) {
            case MISSION_A: {
                // Get the front-facing range data (typically around center of scan)
                int center_index = scan_msg->ranges.size() / 2;
                int angle_width = scan_msg->ranges.size() / 6;  // ~30 degrees on each side
                
                std::vector<float> front_ranges;
                std::vector<float> left_ranges;
                std::vector<float> right_ranges;
                
                // Collect front ranges (~60 degrees arc)
                for (int i = center_index - angle_width; i <= center_index + angle_width; i++) {
                    if (i >= 0 && i < static_cast<int>(scan_msg->ranges.size())) {
                        float range = scan_msg->ranges[i];
                        if (std::isfinite(range) && range > 0.0) {
                            front_ranges.push_back(range);
                        }
                    }
                }
                
                // Collect left ranges (~60 degrees arc)
                for (int i = 0; i < angle_width * 2; i++) {
                    if (i < static_cast<int>(scan_msg->ranges.size())) {
                        float range = scan_msg->ranges[i];
                        if (std::isfinite(range) && range > 0.0) {
                            left_ranges.push_back(range);
                        }
                    }
                }
                
                // Collect right ranges (~60 degrees arc)
                for (int i = scan_msg->ranges.size() - angle_width * 2; i < static_cast<int>(scan_msg->ranges.size()); i++) {
                    if (i >= 0) {
                        float range = scan_msg->ranges[i];
                        if (std::isfinite(range) && range > 0.0) {
                            right_ranges.push_back(range);
                        }
                    }
                }
                
                // Calculate metrics for each region
                float front_mean = 0.0f;
                float left_mean = 0.0f;
                float right_mean = 0.0f;
                
                if (!front_ranges.empty()) {
                    front_mean = std::accumulate(front_ranges.begin(), front_ranges.end(), 0.0f) / front_ranges.size();
                }
                if (!left_ranges.empty()) {
                    left_mean = std::accumulate(left_ranges.begin(), left_ranges.end(), 0.0f) / left_ranges.size();
                }
                if (!right_ranges.empty()) {
                    right_mean = std::accumulate(right_ranges.begin(), right_ranges.end(), 0.0f) / right_ranges.size();
                }
                
                // Calculate the percentage of close points in each region
                float front_close_percent = 0.0f;
                float left_close_percent = 0.0f;
                float right_close_percent = 0.0f;
                
                if (!front_ranges.empty()) {
                    int front_close_count = std::count_if(front_ranges.begin(), front_ranges.end(),
                        [this](float range) { return range < close_threshold_; });
                    front_close_percent = (static_cast<float>(front_close_count) / front_ranges.size()) * 100.0f;
                }
                
                if (!left_ranges.empty()) {
                    int left_close_count = std::count_if(left_ranges.begin(), left_ranges.end(),
                        [this](float range) { return range < close_threshold_; });
                    left_close_percent = (static_cast<float>(left_close_count) / left_ranges.size()) * 100.0f;
                }
                
                if (!right_ranges.empty()) {
                    int right_close_count = std::count_if(right_ranges.begin(), right_ranges.end(),
                        [this](float range) { return range < close_threshold_; });
                    right_close_percent = (static_cast<float>(right_close_count) / right_ranges.size()) * 100.0f;
                }
                
                // Enhanced transition detection logic
                bool narrow_passage_detected = false;
                bool approaching_narrow_passage = false;
                
                // Check if we're between walls (high close percentage on both sides)
                if (left_close_percent > 35.0 && right_close_percent > 35.0) {  // Reduced from 40.0
                    // Also check if there's a clear path forward
                    if (front_close_percent < 30.0 && front_mean > medium_threshold_) {
                        narrow_passage_detected = true;
                    }
                }
                
                // NEW: Early detection of approaching narrow passage
                if (left_ranges.size() > 10 && right_ranges.size() > 10) {
                    // Check if walls are getting closer on both sides
                    if ((left_mean < 1.2 && right_mean < 1.2) &&  // Walls are relatively close
                        (front_mean > medium_threshold_ * 1.2) &&  // Clear path ahead
                        std::abs(left_mean - right_mean) < 0.4) {  // Roughly symmetric walls
                        approaching_narrow_passage = true;
                    }
                }
                
                // Alternative condition: check for parallel walls
                bool parallel_walls = false;
                if (left_ranges.size() > 10 && right_ranges.size() > 10) {
                    // Calculate standard deviation of distances on each side
                    float left_variance = 0.0f, right_variance = 0.0f;
                    
                    for (const auto& range : left_ranges) {
                        left_variance += std::pow(range - left_mean, 2);
                    }
                    left_variance /= left_ranges.size();
                    
                    for (const auto& range : right_ranges) {
                        right_variance += std::pow(range - right_mean, 2);
                    }
                    right_variance /= right_ranges.size();
                    
                    float left_std_dev = std::sqrt(left_variance);
                    float right_std_dev = std::sqrt(right_variance);
                    
                    // If standard deviation is low on both sides and distances are similar,
                    // we're likely between parallel walls
                    if (left_std_dev < 0.15 && right_std_dev < 0.15 &&  // Increased from 0.1
                        std::abs(left_mean - right_mean) < 0.4) {       // Increased from 0.3
                        parallel_walls = true;
                    }
                }
                
                // Log the detailed metrics
                RCLCPP_INFO(this->get_logger(),
                            "Front: mean=%.2fm, close=%.1f%% | Left: mean=%.2fm, close=%.1f%% | Right: mean=%.2fm, close=%.1f%%",
                            front_mean, front_close_percent,
                            left_mean, left_close_percent,
                            right_mean, right_close_percent);
                
                // Transition logic with multiple conditions
                if (narrow_passage_detected || parallel_walls || approaching_narrow_passage) {
                    consecutive_b_detections_++;
                    RCLCPP_INFO(this->get_logger(), "Potential Mission B detection (%d/%d) - %s",
                                consecutive_b_detections_, b_detection_threshold_,
                                approaching_narrow_passage ? "Approaching narrow passage" :
                                (parallel_walls ? "Parallel walls detected" : "Narrow passage detected"));
                    
                    if (consecutive_b_detections_ >= b_detection_threshold_) {
                        RCLCPP_INFO(this->get_logger(), "Transitioning: Mission A -> Mission B");
                        current_mission_ = MISSION_B;
                        consecutive_b_detections_ = 0;
                        publish_mission_state("MISSION_B");
                    }
                } else {
                    // Gradually decrease counter rather than resetting to add hysteresis
                    if (consecutive_b_detections_ > 0) {
                        consecutive_b_detections_--;
                    }
                }
                break;
            }
            
            case MISSION_B: {
                // Enhanced transition detection from B to C (exiting narrow walls)
                // We'll analyze front-facing LiDAR points more heavily
                
                // Get the front-facing range data (typically around center of scan)
                int center_index = scan_msg->ranges.size() / 2;
                int angle_width = scan_msg->ranges.size() / 6; // ~30 degrees on each side
                
                std::vector<float> front_ranges;
                for (int i = center_index - angle_width; i <= center_index + angle_width; i++) {
                    if (i >= 0 && i < static_cast<int>(scan_msg->ranges.size())) {
                        float range = scan_msg->ranges[i];
                        if (std::isfinite(range) && range > 0.0) {
                            front_ranges.push_back(range);
                        }
                    }
                }

                // Calculate front metrics
                float front_mean = 0.0f;
                float front_open_percent = 0.0f;
                
                if (!front_ranges.empty()) {
                    front_mean = std::accumulate(front_ranges.begin(), front_ranges.end(), 0.0f) / front_ranges.size();
                    
                    // Count points that indicate an open space ahead (greater than medium threshold)
                    int front_open_count = 0;
                    for (const auto& range : front_ranges) {
                        if (range > medium_threshold_) {
                            front_open_count++;
                        }
                    }
                    
                    front_open_percent = (static_cast<float>(front_open_count) / front_ranges.size()) * 100.0f;
                    
                    RCLCPP_INFO(this->get_logger(), "Front analysis: mean=%.2fm, open=%.1f%%", 
                                front_mean, front_open_percent);
                }
                
                // Check for disparity specifically around 180 degrees (165-185 degree range)
                bool side_disparity_detected = false;
                float disparity_threshold = 1.0; // Significant jump in distance (meters)
                
                // Calculate indices for the 165-185 degree range
                // With 1080 points over 270 degrees, each degree is ~4 indices
                float degree_to_index = scan_msg->ranges.size() / 270.0f;
                int rear_center_index = static_cast<int>(180 * degree_to_index); // Index for 180 degrees (directly behind)
                int rear_range = static_cast<int>(10 * degree_to_index);         // +/- 10 degrees (165-185 degree range)
                
                // Check for disparities within the rear range (165-185 degrees)
                for (int i = rear_center_index - rear_range; i <= rear_center_index + rear_range; i++) {
                    // Ensure index is within bounds
                    if (i > 0 && i < static_cast<int>(scan_msg->ranges.size()) - 1) {
                        float prev_range = scan_msg->ranges[i-1];
                        float curr_range = scan_msg->ranges[i];
                        if (std::isfinite(prev_range) && std::isfinite(curr_range) && 
                            std::abs(prev_range - curr_range) > disparity_threshold) {
                            
                            // Calculate the approximate angle of this point
                            float angle_deg = (i / degree_to_index);
                            
                            side_disparity_detected = true;
                            RCLCPP_INFO(this->get_logger(), 
                                      "Disparity detected at rear (%.1f degrees): jump of %.2fm", 
                                      angle_deg, std::abs(prev_range - curr_range));
                            break;
                        }
                    }
                }
                
                // Multi-criteria check for exiting narrow passage:
                // 1. Overall scan shows mostly medium & far points
                // 2. Front-facing area shows a high percentage of open space
                // 3. The mean distance in front is relatively high
                // 4. Disparity detected specifically in the rear (165-185 degrees)
                
                if (((medium_percent + far_percent > 65.0) && 
                     (front_open_percent > 80.0) && 
                     (front_mean > medium_threshold_ * 1.5)) || 
                    side_disparity_detected) {
                    
                    consecutive_c_detections_++;
                    RCLCPP_INFO(this->get_logger(), "Potential Mission C detection (%d/%d)%s", 
                                consecutive_c_detections_, c_detection_threshold_,
                                side_disparity_detected ? " - Rear disparity detected!" : "");
                    
                    if (consecutive_c_detections_ >= c_detection_threshold_) {
                        RCLCPP_INFO(this->get_logger(), "*** EDGE DETECTED: Exiting Mission B ***");
                        RCLCPP_INFO(this->get_logger(), "Transitioning: Mission B -> Mission C");
                        current_mission_ = MISSION_C;
                        consecutive_c_detections_ = 0;
                        publish_mission_state("MISSION_C");
                        
                        // Record the position at edge transition if needed
                        // You could add code here to save or transmit the current position
                    }
                } else {
                    // Gradually decrease counter rather than resetting to add hysteresis
                    if (consecutive_c_detections_ > 0) {
                        consecutive_c_detections_--;
                    }
                }
                break;
            }
            
            case MISSION_C: {
                // Keep in Mission C once we've reached it
                // You could add logic here if you need to detect a potential return to Mission A
                break;
            }
        }
        
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
                    
        RCLCPP_INFO(this->get_logger(), "Current mission: %s", 
                    current_mission_ == MISSION_A ? "A (Camera)" : 
                    (current_mission_ == MISSION_B ? "B (Lidar)" : "C (Odometry)"));
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
