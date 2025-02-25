#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <cmath>

class LaneFollowingNode : public rclcpp::Node {
  public:
  LaneFollowingNode()
  : Node("lane_following_node"), previous_error_(0.0), integral_(0.0)
  {
    //Camera parameter
    int camera_index = 3;
    int frame_width = 640;
    int frame_height = 360;

    // Threshold parameters
    thresh = 100;      // simple threshold
    blockSize = 9;     // adaptive threshold
    C = 25;

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
    cap_.open(camera_index);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
    }
    // camera attributes
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap_.set(cv::CAP_PROP_BRIGHTNESS, 128);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&LaneFollowingNode::timer_callback, this));
      cv::namedWindow("Lane Detection", cv::WINDOW_AUTOSIZE);
      cv::namedWindow("Gaussian Blur", cv::WINDOW_AUTOSIZE);
      cv::namedWindow("Canny Edges", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "LaneFollowingNode started.");
  }
  ~LaneFollowingNode() {
    if (cap_.isOpened())
      cap_.release();
    cv::destroyAllWindows();
  }


private:
  double speed_control(double steering_angle) {
    double abs_angle = std::abs(steering_angle);
    double v_max = 1.5;  
    double v_min = 0.7;  
    double k = 30.0;     
    double x0 = 0.2;    
    double sigmoid = 1.0 / (1.0 + std::exp(k * (abs_angle - x0)));
    double speed = v_min + (v_max - v_min) * sigmoid;
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

    int width = frame.cols;
    int height = frame.rows;

    // ROI
    cv::Rect roi_rect(0, height / 2, width, height / 2);
    cv::Mat roi_frame = frame(roi_rect);

    // Grayscale
    cv::Mat gray;
    cv::cvtColor(roi_frame, gray, cv::COLOR_BGR2GRAY);
    
    // Simple threshold
    /*
    cv::Mat binary;
    cv::threshold(gray, binary, thresh, 255, cv::THRESH_BINARY);
    */

    // Adaptive threshold
    cv::Mat binary;
    cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, blockSize, C);

    // Gaussian Blur
    cv::Mat blurred;
    cv::GaussianBlur(binary, blurred, cv::Size(gaus_blur_size, gaus_blur_size), 0);

    // Canny Edge
    cv::Mat edges;
    cv::Canny(blurred, edges, canny_inf, canny_sup);

    // Hough Transform
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, hough_threshold, hough_inf_pixel, hough_pixel_gap);

    // Separate lines
    auto line_pair = separateLine(lines, slope_threshold);
    auto left_lines = line_pair.first;
    auto right_lines = line_pair.second;

    // Weighted Average line detection
    auto left_avg = weighted_average_line(left_lines);
    auto right_avg = weighted_average_line(right_lines);

    // [10] Predict lane coordinates based on the averaged slope and intercept
    //      ROI range: 0 ~ roi_frame.rows (since the ROI starts at y=0)
    int roi_height = roi_frame.rows;
    int y_min = 0;          // top of ROI
    int y_max = roi_height; // bottom of ROI

    cv::Point left_pt1, left_pt2, right_pt1, right_pt2;
    if (!left_lines.empty()) {
      double left_slope = left_avg.first;
      double left_intercept = left_avg.second;
      left_pt1 = cv::Point(static_cast<int>((y_max - left_intercept) / (left_slope + 1e-6)), y_max);
      left_pt2 = cv::Point(static_cast<int>((y_min - left_intercept) / (left_slope + 1e-6)), y_min);
    }
    if (!right_lines.empty()) {
      double right_slope = right_avg.first;
      double right_intercept = right_avg.second;
      right_pt1 = cv::Point(static_cast<int>((y_max - right_intercept) / (right_slope + 1e-6)), y_max);
      right_pt2 = cv::Point(static_cast<int>((y_min - right_intercept) / (right_slope + 1e-6)), y_min);
    }

    int lane_center_x = width / 2; 
    if (!left_lines.empty() && !right_lines.empty()) {
      lane_center_x = (left_pt1.x + right_pt1.x) / 2;
    } else if (!left_lines.empty()) {
      lane_center_x = left_pt1.x + width / 2;
    } else if (!right_lines.empty()) {
      lane_center_x = right_pt1.x - width / 2;
    }

    // [12] Visualization: draw lines on the original image (with ROI offset)
    cv::Mat laneVis = frame.clone();
    int offset_y = roi_rect.y; // offset to match original image coordinates

    // Draw left line in blue
    if (!left_lines.empty()) {
      cv::line(
        laneVis,
        cv::Point(left_pt1.x, left_pt1.y + offset_y),
        cv::Point(left_pt2.x, left_pt2.y + offset_y),
        cv::Scalar(255, 0, 0), 3
      );
    }
    // Draw right line in green
    if (!right_lines.empty()) {
      cv::line(
        laneVis,
        cv::Point(right_pt1.x, right_pt1.y + offset_y),
        cv::Point(right_pt2.x, right_pt2.y + offset_y),
        cv::Scalar(0, 255, 0), 3
      );
    }

    // Draw lane center 
    cv::circle(laneVis, cv::Point(lane_center_x, height - 1), 10, cv::Scalar(255, 0, 0), -1);

    // Display windows
    cv::imshow("Lane Detection", laneVis);
    cv::imshow("Gaussian Blur", blurred);
    cv::imshow("Canny Edges", edges);
    cv::waitKey(1);

    // [13] Simple proportional control (P-control)
    double error = static_cast<double>(lane_center_x) - (width / 2.0);
    integral_ += error;
    double derivative = error - previous_error_;
    double steering = -(Kp * error + Ki * integral_ + Kd * derivative) / 3;
    previous_error_ = error;
    double drive_speed = 0.0;
    
    drive_speed = speed_control(steering);

    // Publish steering commands
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = this->now();
    drive_msg.drive.steering_angle = steering;
    drive_msg.drive.speed = drive_speed;
    drive_pub_->publish(drive_msg);

  }
  
  // Member variables
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
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaneFollowingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
  }


