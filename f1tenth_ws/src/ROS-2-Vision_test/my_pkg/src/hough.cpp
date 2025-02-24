#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>
#include <numeric>
#include <cstdlib>
#include <ctime>
#include <cmath>

class LaneFollowerPolyNode : public rclcpp::Node {
public:
  LaneFollowerPolyNode()
  : Node("lane_follower_node"), previous_error_(0.0), integral_(0.0)
  {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    // PID parameters
    this->declare_parameter<double>("Kp", 0.01);
    this->declare_parameter<double>("Ki", 0.00);
    this->declare_parameter<double>("Kd", 0.00);
    this->get_parameter("Kp", Kp_);
    this->get_parameter("Ki", Ki_);
    this->get_parameter("Kd", Kd_);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 10,
      std::bind(&LaneFollowerPolyNode::image_callback, this, std::placeholders::_1));

    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", 10);
    RCLCPP_INFO(this->get_logger(), "LaneFollowerNode has been started.");
  }

private:

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch(cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat frame = cv_ptr->image;
    int height = frame.rows;
    int width = frame.cols;
    
    //ROI
    cv::Rect roi_rect(0, height / 2, width, height / 2);
    cv::Mat roi = frame(roi_rect);
    
    // Grayscale
    cv::Mat Gray;
    cv::cvtColor(roi, Gray, cv::COLOR_BGR2GRAY);

    // equalize histogram 
    //cv::Mat Hist;
    //cv::equalizeHist(Gray, Hist);
    
    // simple threshold
    /*
    cv::Mat binary;
    int threshold = 100;
    cv::threshold(Gray, binary, threshold, 255, cv::THRESH_BINARY);
    */
    
    // adaptive threshold
    cv::Mat binary;
    int blockSize = 31;
    int C = 20;
    cv::adaptiveThreshold(Gray, binary, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, blockSize, C);
    
    //black lane
    cv::Mat inverted;
    cv::bitwise_not(binary, inverted);
    
    // Gaussian blur
    cv::Mat blurred;
    cv::GaussianBlur(inverted, blurred, cv::Size(5, 5), 0);
    
    //canny edge
    cv::Mat edges;
    cv::Canny(blurred, edges, 10, 150);
    
    // Hough
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 80, 100, 5);
    
    // left right seperate
    std::vector<cv::Vec4i> left_lines, right_lines;
    
    double slope_threshold = 0.3; 
    
    for (const auto &line : lines) {
      int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
      double slope = static_cast<double>(y2 - y1) / (x2 - x1 + 1e-6);
      if (std::abs(slope) < slope_threshold)
        continue;
      if (slope < 0)
        left_lines.push_back(line);
      else
        right_lines.push_back(line);
    }

    // lane calculate
    auto average_line = [&](const std::vector<cv::Vec4i>& lines) -> cv::Vec4i {
      int x1_sum = 0, y1_sum = 0, x2_sum = 0, y2_sum = 0;
      for (const auto &line : lines) {
        x1_sum += line[0];
        y1_sum += line[1];
        x2_sum += line[2];
        y2_sum += line[3];
      }
      int count = lines.size();
      if (count == 0)
        return cv::Vec4i(0, 0, 0, 0);
      return cv::Vec4i(x1_sum / count, y1_sum / count, x2_sum / count, y2_sum / count);
    };

    cv::Vec4i left_avg = average_line(left_lines);
    cv::Vec4i right_avg = average_line(right_lines);


    // lane coordinate
    int roi_height = roi.rows;
    int y_min = static_cast<int>(roi_height * 0.0); 
    int y_max = roi_height;                         

    cv::Point left_pt1, left_pt2, right_pt1, right_pt2;
    if (!left_lines.empty()) {
      double left_slope = static_cast<double>(left_avg[3] - left_avg[1]) / (left_avg[2] - left_avg[0] + 1e-6);
      double left_intercept = left_avg[1] - left_slope * left_avg[0];
      left_pt1 = cv::Point(static_cast<int>((y_max - left_intercept) / left_slope), y_max);
      left_pt2 = cv::Point(static_cast<int>((y_min - left_intercept) / left_slope), y_min);
    }
    if (!right_lines.empty()) {
      double right_slope = static_cast<double>(right_avg[3] - right_avg[1]) / (right_avg[2] - right_avg[0] + 1e-6);
      double right_intercept = right_avg[1] - right_slope * right_avg[0];
      right_pt1 = cv::Point(static_cast<int>((y_max - right_intercept) / right_slope), y_max);
      right_pt2 = cv::Point(static_cast<int>((y_min - right_intercept) / right_slope), y_min);
    }

    // lane center
    int lane_center_x = width / 2; // 기본값은 영상 중앙
    if (!left_lines.empty() && !right_lines.empty()) {
      lane_center_x = (left_pt1.x + right_pt1.x) / 2;
    } else if (!left_lines.empty()) {
      lane_center_x = left_pt1.x + (width - left_pt1.x) / 2;
    } else if (!right_lines.empty()) {
      lane_center_x = right_pt1.x / 2;
    }

    // visualizing
    cv::Mat laneVis = frame.clone();
    int offset_y = roi_rect.y; // ROI가 원본 영상 내에서 시작하는 y 좌표
    if (!left_lines.empty()) {
      cv::line(laneVis, cv::Point(left_pt1.x, left_pt1.y + offset_y),
               cv::Point(left_pt2.x, left_pt2.y + offset_y), cv::Scalar(255, 0, 0), 3);
    }
    if (!right_lines.empty()) {
      cv::line(laneVis, cv::Point(right_pt1.x, right_pt1.y + offset_y),
               cv::Point(right_pt2.x, right_pt2.y + offset_y), cv::Scalar(0, 255, 0), 3);
    }
    cv::line(laneVis, cv::Point(lane_center_x, offset_y + y_max),
             cv::Point(lane_center_x, offset_y + y_min), cv::Scalar(0, 0, 255), 2);
    cv::imshow("Lane Detection", laneVis);
    
    cv::waitKey(1);
    /*auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = this->now();
    drive_msg.drive.steering_angle = steering;
    drive_pub_->publish(drive_msg);
*/
    // visualize
    cv::imshow("frame", frame);
    cv::imshow("Canny", edges);
    cv::imshow("binary", inverted);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  double Kp_, Ki_, Kd_;
  double previous_error_;
  double integral_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaneFollowerPolyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}
