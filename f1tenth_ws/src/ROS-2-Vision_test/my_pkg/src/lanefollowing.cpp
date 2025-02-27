#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <vector>
#include <cmath>
#include <chrono>

class LaneFollowingNode : public rclcpp::Node {
  public:
  LaneFollowingNode()
  : Node("lane_following_node"), previous_error_(0.0), integral_(0.0)
  {
    // Camera parameter
    int camera_index = 3;
    
    std::system("v4l2-ctl -c auto_exposure=1 -d /dev/video3");
    std::system("v4l2-ctl -d /dev/video3 --set-fmt-video=width=1280,height=720,pixelformat=MJPG");
    std::system("v4l2-ctl -d /dev/video3 --set-parm=60");
    cap_.open(camera_index, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
    }
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap_.set(cv::CAP_PROP_FPS, 60);
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    double width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    double height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap_.get(cv::CAP_PROP_FPS);
    std::cout << "Camera settings: " << width << "x" << height << " at " << fps << " fps." << std::endl;
    // camera attributes
    //cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
    //cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);

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
  
    rclcpp::Time current_time = this->now();
    double diff_sec = (current_time - prev_time_).seconds() * 1000.0 - 16.67;
    RCLCPP_INFO(this->get_logger(), "Time difference: %f ms", diff_sec);
    prev_time_ = current_time;
    
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
    {
      cv::cuda::GpuMat gpu_binary, gpu_blurred;
      gpu_binary.upload(binary);
      cv::Ptr<cv::cuda::Filter> gaussFilter = cv::cuda::createGaussianFilter(gpu_binary.type(), gpu_binary.type(), cv::Size(gaus_blur_size, gaus_blur_size), 0);
      gaussFilter->apply(gpu_binary, gpu_blurred);
      gpu_blurred.download(blurred);
    }

    // Canny Edge
    cv::Mat edges;
    {
      cv::cuda::GpuMat gpu_blurred, gpu_edges;
      gpu_blurred.upload(blurred);
      cv::Ptr<cv::cuda::CannyEdgeDetector> canny = cv::cuda::createCannyEdgeDetector(canny_inf, canny_sup);
      canny->detect(gpu_blurred, gpu_edges);
      gpu_edges.download(edges);
    }

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

    // lane coordinates
    int roi_height = roi_frame.rows;
    int y_min = 0;
    int y_max = roi_height;

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

    // Visualization
    cv::Mat laneVis = frame.clone();
    int offset_y = roi_rect.y;
    
    if (!left_lines.empty()) {
      cv::line(laneVis, cv::Point(left_pt1.x, left_pt1.y + offset_y), cv::Point(left_pt2.x, left_pt2.y + offset_y), cv::Scalar(255, 0, 0), 3);
    }
    if (!right_lines.empty()) {
      cv::line(laneVis, cv::Point(right_pt1.x, right_pt1.y + offset_y), cv::Point(right_pt2.x, right_pt2.y + offset_y), cv::Scalar(0, 255, 0), 3);
    }
    cv::circle(laneVis, cv::Point(lane_center_x, height - 1), 10, cv::Scalar(255, 0, 0), -1);

    // Display
    cv::imshow("Lane Detection", laneVis);
    cv::imshow("Masked", edges);
    cv::waitKey(1);
    
    // PID control
    double error = static_cast<double>(lane_center_x) - (width / 2.0);
    integral_ += error;
    double derivative = error - previous_error_;
    double steering = -(Kp * error + Ki * integral_ + Kd * derivative) / 3;
    previous_error_ = error;
    double drive_speed = 0.0;
    
    drive_speed = speed_control(steering);

    // Publish
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = this->now();
    drive_msg.drive.steering_angle = steering;
    drive_msg.drive.speed = drive_speed;
    drive_pub_->publish(drive_msg);
  }
  
  // Member variables
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
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaneFollowingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
