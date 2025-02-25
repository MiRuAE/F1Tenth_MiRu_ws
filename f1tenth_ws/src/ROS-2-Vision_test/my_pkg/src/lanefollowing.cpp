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
  LaneFollowingNode() : Node("lane_following_node") {
    // Subscribe to camera image
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 10,
      std::bind(&LaneFollowingNode::image_callback, this, std::placeholders::_1));
    
    // Publisher for Ackermann steering commands
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", 10);

    cv::namedWindow("Lane Detection", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "LaneFollowingNode started.");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // [0] Convert ROS image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat frame = cv_ptr->image;
    int width = frame.cols;
    int height = frame.rows;

    // [1] Define ROI: Take the lower half of the original image
    cv::Rect roi_rect(0, height / 2, width, height / 2);
    cv::Mat roi_frame = frame(roi_rect);

    // [2] Convert ROI to grayscale
    cv::Mat gray;
    cv::cvtColor(roi_frame, gray, cv::COLOR_BGR2GRAY);

    // [3] (Optional) Histogram equalization for contrast enhancement (commented out)
    // cv::Mat equalized;
    // cv::equalizeHist(gray, equalized);

    // [4] Thresholding (Using Adaptive Threshold in this example)
    
    // cv::Mat binary;
    // double thresh = 100;
    // cv::threshold(gray, binary, thresh, 255, cv::THRESH_BINARY);
    
    cv::Mat binary;
    int blockSize = 9; // Must be an odd number
    int C = 25;        // Constant subtracted from the mean
    cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                          cv::THRESH_BINARY, blockSize, C);

    // [5] Gaussian Blur to reduce noise
    cv::Mat blurred;
    cv::GaussianBlur(binary, blurred, cv::Size(5, 5), 0);

    // [6] Canny Edge Detection
    cv::Mat edges;
    cv::Canny(blurred, edges, 50, 150);

    // [7] Hough Transform to detect lines
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);

    // [8] Separate detected lines into left and right based on slope
    //     Ignore nearly horizontal lines
    std::vector<cv::Vec4i> left_lines, right_lines;
    double slope_threshold = 0.3;
    for (const auto &line : lines) {
      int x1 = line[0], y1 = line[1];
      int x2 = line[2], y2 = line[3];
      double slope = static_cast<double>(y2 - y1) / (x2 - x1 + 1e-6);

      // Ignore lines whose slope is almost 0
      if (std::abs(slope) < slope_threshold) {
        continue;
      }

      // Negative slope -> left lane, positive slope -> right lane
      if (slope < 0)
        left_lines.push_back(line);
      else
        right_lines.push_back(line);
    }

    // [9] Weighted Average function for lines
    //     Each line's slope/intercept is weighted by its length
    auto weighted_average_line = [&](const std::vector<cv::Vec4i>& lines_vec) {
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

        // Calculate the line length (used as a weight)
        double length = std::sqrt((x2 - x1) * (x2 - x1) +
                                  (y2 - y1) * (y2 - y1));

        slope_sum    += slope * length;
        intercept_sum += intercept * length;
        length_sum   += length;
      }

      // If no lines exist, return 0
      if (length_sum == 0.0) {
        return std::make_pair(0.0, 0.0);
      }

      double avg_slope = slope_sum / length_sum;
      double avg_intercept = intercept_sum / length_sum;
      return std::make_pair(avg_slope, avg_intercept);
    };

    // Compute weighted average for left/right lines
    auto left_avg = weighted_average_line(left_lines);
    auto right_avg = weighted_average_line(right_lines);

    // [10] Predict lane coordinates based on the averaged slope and intercept
    //      ROI range: 0 ~ roi_frame.rows (since the ROI starts at y=0)
    int roi_height = roi_frame.rows;
    int y_min = 0;          // top of ROI
    int y_max = roi_height; // bottom of ROI

    cv::Point left_pt1, left_pt2, right_pt1, right_pt2;

    // If left lines exist, compute representative line
    if (!left_lines.empty()) {
      double left_slope = left_avg.first;
      double left_intercept = left_avg.second;
      left_pt1 = cv::Point(
        static_cast<int>((y_max - left_intercept) / (left_slope + 1e-6)), y_max
      );
      left_pt2 = cv::Point(
        static_cast<int>((y_min - left_intercept) / (left_slope + 1e-6)), y_min
      );
    }
    // If right lines exist, compute representative line
    if (!right_lines.empty()) {
      double right_slope = right_avg.first;
      double right_intercept = right_avg.second;
      right_pt1 = cv::Point(
        static_cast<int>((y_max - right_intercept) / (right_slope + 1e-6)), y_max
      );
      right_pt2 = cv::Point(
        static_cast<int>((y_min - right_intercept) / (right_slope + 1e-6)), y_min
      );
    }

    // [11] Determine the lane center
    //      If both lines exist, use the average of their bottom x-coordinates
    int lane_center_x = width / 2; // default to image center
    if (!left_lines.empty() && !right_lines.empty()) {
      lane_center_x = (left_pt1.x + right_pt1.x) / 2;
    } else if (!left_lines.empty()) {
      // If only left line, assume the lane extends to the right edge
      lane_center_x = left_pt1.x + (width - left_pt1.x) / 2;
    } else if (!right_lines.empty()) {
      // If only right line, assume the lane extends from the left edge
      lane_center_x = right_pt1.x / 2;
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

    // Draw lane center line in red
    cv::line(
      laneVis,
      cv::Point(lane_center_x, offset_y + y_max),
      cv::Point(lane_center_x, offset_y + y_min),
      cv::Scalar(0, 0, 255), 2
    );

    // Display windows
    cv::imshow("Lane Detection", laneVis);
    cv::imshow("Gaussian Blur", blurred);
    cv::imshow("Canny Edges", edges);
    cv::waitKey(1);

    // [13] Simple proportional control (P-control)
    int image_center = width / 2;
    int error = lane_center_x - image_center;
    double Kp = 0.005;
    double steering_angle = -Kp * error / 2; // negative sign for correction
    double drive_speed = 1.0;

    // Publish steering commands
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = this->now();
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = drive_speed;
    drive_pub_->publish(drive_msg);
  }

  // Member variables
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaneFollowingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
  }
