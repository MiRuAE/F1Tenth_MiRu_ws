#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <cmath>
#include <chrono>
#include <std_msgs/msg/string.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class LaneFollowingNode : public rclcpp::Node {
  public:
  LaneFollowingNode()
  : Node("lane_following_node"), previous_error_(0.0), integral_(0.0), is_active_(true)
  {
    // Mission state subscriber
    mission_sub_ = this->create_subscription<std_msgs::msg::String>(
      "current_mission", 10,
      std::bind(&LaneFollowingNode::mission_callback, this, std::placeholders::_1));
      
    // Camera parameter
    int camera_index = 0;
    cap_.open(camera_index, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
    }
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
    cap_.set(cv::CAP_PROP_FPS, 30);
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap_.set(cv::CAP_PROP_BRIGHTNESS, 128);
    double width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    double height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap_.get(cv::CAP_PROP_FPS);
    std::cout << "Camera settings: " << width << "x" << height << " at " << fps << " fps." << std::endl;
    // camera attributes
    //cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
    //cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);

    // Threshold parameters
    thresh = 100;      // simple threshold
    blockSize = 11;     // adaptive threshold
    C = 10;

    // Gaussian blur parameter
    gaus_blur_size = 5;

    // Canny edge parameters
    canny_inf = 50;
    canny_sup = 150;

    // Hough Transform parameters
    hough_threshold = 50;
    hough_inf_pixel = 50;
    hough_pixel_gap = 10;

    // Line detection parameter
    slope_threshold = 0.3;

    // PID parameters
    Kp = 0.005;
    Ki = 0.0;
    Kd = 0.0;

    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    
    // Add marker publisher for rviz visualization
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lane_center_marker", 10);
    
    // Add path publisher for trajectory visualization
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("camera_path", 10);
    
    prev_time_ = this->now();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(15),
      std::bind(&LaneFollowingNode::timer_callback, this));
      cv::namedWindow("Lane Detection", cv::WINDOW_AUTOSIZE);
      cv::namedWindow("Masked", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "LaneFollowingNode started.");
  }
  ~LaneFollowingNode() {
    if (cap_.isOpened())
      cap_.release();
    cv::destroyAllWindows();
  }


private:
  void mission_callback(const std_msgs::msg::String::SharedPtr msg) {
    bool was_active = is_active_;
    is_active_ = (msg->data == "MISSION_A");
    
    if (is_active_ != was_active) {
      if (is_active_) {
        RCLCPP_INFO(this->get_logger(), "Camera node activated - Mission A");
      } else {
        RCLCPP_INFO(this->get_logger(), "Camera node deactivated");
        
        // Delete marker when deactivating
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "lane_center";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_pub_->publish(marker);

        // Clear path when deactivating
        path_.poses.clear();
        path_.header.frame_id = "base_link";
        path_.header.stamp = this->now();
        path_pub_->publish(path_);
      }
    }
  }

  double speed_control(double slope) {
    double k = 1.0;     
    double x0 = 3.0;    
    double sigmoid = 1.0 / (1.0 + std::exp(k * (slope - x0)));
    double speed = 2.0 - 1.3 * sigmoid;
    return speed;
  }

  std::pair<std::vector<cv::Vec4i>, std::vector<cv::Vec4i>>
    separateLine(const std::vector<cv::Vec4i>& lines, double slope_threshold) {
      std::vector<cv::Vec4i> left_lines;
      std::vector<cv::Vec4i> right_lines;
      for (const auto &line : lines) {
        int x1 = line[0], y1 = line[1];
        int x2 = line[2], y2 = line[3];
        double slope = static_cast<double>(y2 - y1) / (x2 - x1 + 1e-6);
        if (std::abs(slope) < slope_threshold) {
          continue;
        }
        if (slope < 0)
          left_lines.push_back(line);
        else
          right_lines.push_back(line);
      }
      return std::make_pair(left_lines, right_lines);
    }

  std::pair<double, double> weighted_average_line(const std::vector<cv::Vec4i>& lines_vec) {
    double slope_sum = 0.0;
    double intercept_sum = 0.0;
    double length_sum = 0.0;

    for (const auto &l : lines_vec) {
      double x1 = l[0];
      double y1 = l[1];
      double x2 = l[2];
      double y2 = l[3];

      double slope = (y2 - y1) / (x2 - x1 + 1e-6);
      double intercept = y1 - slope * x1;
      double length = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

      slope_sum += slope * length;
      intercept_sum += intercept * length;
      length_sum += length;
    }

    if (length_sum == 0.0) {
      return std::make_pair(0.0, 0.0);
    }

    double avg_slope = slope_sum / length_sum;
    double avg_intercept = intercept_sum / length_sum;
    return std::make_pair(avg_slope, avg_intercept);
  }

  void timer_callback() {
    cv::Mat frame;
    if (!cap_.read(frame)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to capture frame!");
      return;
    }
    
    rclcpp::Time start = this->now();
    int width = frame.cols;
    int height = frame.rows;

    // ROI
    cv::Rect roi_rect(0, height / 2, width, height / 2);
    cv::Mat roi_frame = frame(roi_rect);

    // Grayscale
    cv::Mat gray;
    cv::cvtColor(roi_frame, gray, cv::COLOR_BGR2GRAY);
    
    cv::Mat inverted;
    cv::bitwise_not(gray, inverted);

    // Gaussian Blur
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(gaus_blur_size, gaus_blur_size), 0);
    
    // Simple threshold
    /*
    cv::Mat binary;
    cv::threshold(gray, binary, thresh, 255, cv::THRESH_BINARY);
    */

    
    // Adaptive threshold
    cv::Mat binary;
    cv::adaptiveThreshold(blurred, binary, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, blockSize, C);
    
    /*
    // Otsu threshold
    cv::Mat binary;
    double thresh_val = cv::threshold(blurred, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    //RCLCPP_INFO(this->get_logger(), "%f", thresh_val);
    
    cv::Mat otsu_adap;
    cv::adaptiveThreshold(blurred, otsu_adap, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, blockSize, (128 - thresh_val) / 10);
    */

    // Canny Edge
    cv::Mat edges;
    cv::Canny(binary, edges, canny_inf, canny_sup);
    
    int half_height = edges.rows;
    
    cv::Rect velocity_roi(0, 0, width, half_height / 3);
    cv::Rect lanecencter_roi(0, half_height / 3, width, half_height * 2 / 3);
    
    cv::Mat roi_v = edges(velocity_roi);
    cv::Mat roi_c = edges(lanecencter_roi);
    
    // Hough Transform
    std::vector<cv::Vec4i> lines_v;
    std::vector<cv::Vec4i> lines_c;
    cv::HoughLinesP(roi_v, lines_v, 1, CV_PI / 180, hough_threshold, hough_inf_pixel, hough_pixel_gap);
    cv::HoughLinesP(roi_c, lines_c, 1, CV_PI / 180, hough_threshold, hough_inf_pixel, hough_pixel_gap);

    // Separate lines
    auto line_pair_v = separateLine(lines_v, slope_threshold);
    auto left_lines_v = line_pair_v.first;
    auto right_lines_v = line_pair_v.second;
    
    auto line_pair_c = separateLine(lines_c, slope_threshold);
    auto left_lines_c = line_pair_c.first;
    auto right_lines_c = line_pair_c.second;

    // Weighted Average line detection
    auto left_avg_v = weighted_average_line(left_lines_v);
    auto right_avg_v = weighted_average_line(right_lines_v);
    
    auto left_avg_c = weighted_average_line(left_lines_c);
    auto right_avg_c = weighted_average_line(right_lines_c);

    // lane center calculate
    int roi_height = roi_c.rows;
    int y_min = 0;
    int y_max = roi_height;

    cv::Point left_pt1, left_pt2, right_pt1, right_pt2;
    if (!left_lines_c.empty()) {
      double left_slope = left_avg_c.first;
      double left_intercept = left_avg_c.second;
      left_pt1 = cv::Point(static_cast<int>((y_max - left_intercept) / (left_slope + 1e-6)), y_max);
      left_pt2 = cv::Point(static_cast<int>((y_min - left_intercept) / (left_slope + 1e-6)), y_min);
    }
    if (!right_lines_c.empty()) {
      double right_slope = right_avg_c.first;
      double right_intercept = right_avg_c.second;
      right_pt1 = cv::Point(static_cast<int>((y_max - right_intercept) / (right_slope + 1e-6)), y_max);
      right_pt2 = cv::Point(static_cast<int>((y_min - right_intercept) / (right_slope + 1e-6)), y_min);
    }

    int lane_center_x = width / 2; 
    if (!left_lines_c.empty() && !right_lines_c.empty()) {
      lane_center_x = (left_pt1.x + right_pt1.x) / 2;
    } else if (!left_lines_c.empty()) {
      lane_center_x = left_pt1.x + width / 2;
    } else if (!right_lines_c.empty()) {
      lane_center_x = right_pt1.x - width / 2;
    }
    
    
    double max_slope = 30.0;
    
    //roi_v slope
    if (!left_lines_v.empty() && !right_lines_v.empty()) {
      max_slope = 2 / (left_avg_v.first + right_avg_v.first);
    } else if (!left_lines_v.empty()) {
      max_slope = std::abs(left_avg_v.first);
    } else if (!right_lines_v.empty()) {
      max_slope = std::abs(right_avg_v.first);
    }
    
    
    RCLCPP_INFO(this->get_logger(), "%0.5f", max_slope);
    
    
    
    // Visualization
    
    cv::Mat laneVis = frame.clone();
    int offset_y = lanecencter_roi.y + height / 2;
    
    if (!left_lines_c.empty()) {
      cv::line(laneVis, cv::Point(left_pt1.x, left_pt1.y + offset_y), cv::Point(left_pt2.x, left_pt2.y + offset_y), cv::Scalar(255, 0, 0), 3);
    }
    if (!right_lines_c.empty()) {
      cv::line(laneVis, cv::Point(right_pt1.x, right_pt1.y + offset_y), cv::Point(right_pt2.x, right_pt2.y + offset_y), cv::Scalar(0, 255, 0), 3);
    }
    cv::circle(laneVis, cv::Point(lane_center_x, height - 1), 10, cv::Scalar(255, 0, 0), -1);

    // Display
    cv::imshow("Lane Detection", laneVis);
    //cv::imshow("Filtered raw", binary);
    cv::imshow("roi_c", roi_c);
    cv::imshow("roi_v", roi_v);
    cv::waitKey(1);
    
    // PID control
    double error = static_cast<double>(lane_center_x) - (width / 2.0);
    integral_ += error;
    double derivative = error - previous_error_;
    double steering = -(Kp * error + Ki * integral_ + Kd * derivative) / 3;
    previous_error_ = error;
    double drive_speed = 0.0;
    
    drive_speed = speed_control(max_slope);

    // Only publish drive command if in sector A
    if (is_active_) {
      // Publish lane center marker
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = this->now();
      marker.ns = "lane_center";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      
      // Convert pixel coordinates to 3D space
      double pixel_to_meter = 0.001;  // 픽셀을 미터로 변환하는 스케일
      
      // Calculate x, y coordinates in meters
      double x = 0.5;  // 차량 앞쪽 0.5m 지점
      double y = (lane_center_x - width/2) * pixel_to_meter;  // 중앙에서의 오프셋
      
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;
      
      marker.scale.x = 0.1;  // 10cm diameter
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      
      marker_pub_->publish(marker);

      // Add current position to path
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "base_link";
      pose.header.stamp = this->now();
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      
      path_.poses.push_back(pose);
      
      // Limit path size to prevent memory issues
      if (path_.poses.size() > 1000) {
        path_.poses.erase(path_.poses.begin());
      }
      
      // Publish path
      path_.header.frame_id = "base_link";
      path_.header.stamp = this->now();
      path_pub_->publish(path_);

      auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
      drive_msg.header.stamp = this->now();
      drive_msg.drive.steering_angle = steering;
      drive_msg.drive.speed = drive_speed;
      drive_pub_->publish(drive_msg);
    }

    rclcpp::Time end = this->now();
    RCLCPP_INFO(this->get_logger(), "Time: %f ms", (end.seconds() - start.seconds()) * 1000);
  }
  // Member variables
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  bool is_active_;
  rclcpp::Time prev_time_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
  double Kp, Ki, Kd;
  double previous_error_;
  double integral_;

  // Parameters
  int thresh;
  int blockSize;
  int C;
  int gaus_blur_size;
  int canny_inf, canny_sup;
  int hough_threshold, hough_inf_pixel, hough_pixel_gap;
  double slope_threshold;
  
  // Add marker publisher to member variables
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Add new member variables
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaneFollowingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
  }
