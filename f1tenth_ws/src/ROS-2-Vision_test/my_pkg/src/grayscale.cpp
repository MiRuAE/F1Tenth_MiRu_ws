#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <algorithm>

class LaneFollowerNode : public rclcpp::Node {
public:
  LaneFollowerNode()
  : Node("lane_follower_node"), previous_error_(0.0), integral_(0.0)
  {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    // PID 파라미터 선언 및 초기화
    this->declare_parameter<double>("Kp", 0.005);
    this->declare_parameter<double>("Ki", 0.000);
    this->declare_parameter<double>("Kd", 0.000);
    this->get_parameter("Kp", Kp_);
    this->get_parameter("Ki", Ki_);
    this->get_parameter("Kd", Kd_);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 10,
      std::bind(&LaneFollowerNode::image_callback, this, std::placeholders::_1));

    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", 10);

    cv::namedWindow("Lane Tracking", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("ROI Edges", cv::WINDOW_AUTOSIZE);

    // Kalman
    float dt = 0.1;
    kalman_filter_ = cv::KalmanFilter(2, 1, 0);
    kalman_filter_.transitionMatrix = (cv::Mat_<float>(2, 2) << 1, dt, 0, 1);
    kalman_filter_.measurementMatrix = (cv::Mat_<float>(1, 2) << 1, 0);
    cv::setIdentity(kalman_filter_.processNoiseCov, cv::Scalar::all(1e-2));
    cv::setIdentity(kalman_filter_.measurementNoiseCov, cv::Scalar::all(1e-2));
    cv::setIdentity(kalman_filter_.errorCovPost, cv::Scalar::all(1));
    kalman_filter_.statePost = cv::Mat::zeros(2, 1, CV_32F);

    RCLCPP_INFO(this->get_logger(), "LaneFollowerNode started.");
  }

private:
  // RANSAC을 이용해 두께를 포함한 한 차선을 검출하는 함수
  // 반복 횟수(iterations): 100, 거리 임계값(threshold): 20.0, 최소 inlier 수(minInliers): 500
  bool ransacLine(const std::vector<cv::Point>& points, cv::Vec4f &best_line,
                  std::vector<cv::Point> &best_inliers, int iterations = 100,
                  double threshold = 20.0, int minInliers = 500) {
    size_t best_count = 0;  // size_t 사용
    if (points.size() < 2)
      return false;
    for (int i = 0; i < iterations; i++) {
      int idx1 = std::rand() % points.size();
      int idx2 = std::rand() % points.size();
      if (idx1 == idx2)
        continue;
      cv::Point p1 = points[idx1];
      cv::Point p2 = points[idx2];
      double dx = p2.x - p1.x;
      double dy = p2.y - p1.y;
      if (std::hypot(dx, dy) < 1e-6)
        continue;
      double norm = std::sqrt(dx * dx + dy * dy);
      double vx = dx / norm;
      double vy = dy / norm;
      cv::Vec4f line_candidate(vx, vy, static_cast<float>(p1.x), static_cast<float>(p1.y));
      std::vector<cv::Point> candidate_inliers;
      // 각 점과 후보 직선 사이의 거리를 계산하여 임계값보다 작으면 inlier로 추가
      for (const auto &pt : points) {
        double distance = std::abs(vy * (pt.x - p1.x) - vx * (pt.y - p1.y));
        if (distance < threshold)
          candidate_inliers.push_back(pt);
      }
      if (candidate_inliers.size() > best_count) {
        best_count = candidate_inliers.size();
        best_line = line_candidate;
        best_inliers = candidate_inliers;
      }
    }
    return best_count >= static_cast<size_t>(minInliers);
  }

  // 주어진 직선(벡터: (vx, vy, x0, y0))에서 특정 y 좌표에 대응하는 x 좌표 계산
  int lineXAtY(const cv::Vec4f &line, int y) {
    float vx = line[0], vy = line[1], x0 = line[2], y0 = line[3];
    if (std::abs(vy) < 1e-6)
      return static_cast<int>(x0);
    return static_cast<int>(x0 + vx * ((y - y0) / vy));
  }

  // 이미지에 직선을 그리는 함수
  void lane_visualize_line(cv::Mat &img, const cv::Vec4f &line, cv::Scalar color, int height) {
    int y1 = height / 2;
    int y2 = height - 1;
    int x1 = lineXAtY(line, y1);
    int x2 = lineXAtY(line, y2);
    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat frame = cv_ptr->image;
    int height = frame.rows;
    int width = frame.cols;

    // ROI: 이미지 하단 절반 영역
    cv::Rect roi_rect(0, height / 2, width, height / 2);
    cv::Mat roi = frame(roi_rect);

    // 그레이스케일 변환 및 adaptive threshold 적용
    cv::Mat Gray;
    cv::cvtColor(roi, Gray, cv::COLOR_BGR2GRAY);
    cv::Mat binary;
    int blockSize = 9;
    int C = 25;
    cv::adaptiveThreshold(Gray, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY, blockSize, C);

    // 이진화 반전: 차선(검은색)을 강조
    cv::Mat inverted;
    cv::bitwise_not(binary, inverted);

    // 가우시안 블러 후 Canny edge 검출
    cv::Mat blurred;
    cv::GaussianBlur(inverted, blurred, cv::Size(9, 9), 0);
    cv::Mat edges;
    cv::Canny(blurred, edges, 10, 150);

    // ROI 내의 edge 포인트 추출 (원래 이미지 좌표로 변환)
    std::vector<cv::Point> roiPoints;
    cv::findNonZero(edges, roiPoints);
    for (auto &pt : roiPoints) {
      pt.y += roi.rows;
    }

    cv::Mat visFrame = frame.clone();
    std::vector<cv::Point> allPoints = roiPoints;
    cv::Vec4f line1, line2;
    std::vector<cv::Point> inliers1, inliers2;
    bool found_line1 = ransacLine(allPoints, line1, inliers1, 100, 20.0, 500);
    bool found_line2 = false;
    if (found_line1) {
      // 첫 번째 선의 inlier 제거 후 나머지 점들로 두 번째 선 검출
      std::vector<cv::Point> remainingPoints;
      for (const auto &pt : allPoints) {
        if (std::find(inliers1.begin(), inliers1.end(), pt) == inliers1.end())
          remainingPoints.push_back(pt);
      }
      found_line2 = ransacLine(remainingPoints, line2, inliers2, 100, 20.0, 500);
    }

    // 검출된 두 선을 이미지 하단에서의 x 좌표 기준으로 좌측/우측 분류
    bool leftFound = false, rightFound = false;
    cv::Vec4f left_line, right_line;
    int bottomY = height - 1;
    if (found_line1 && found_line2) {
      int x1 = lineXAtY(line1, bottomY);
      int x2 = lineXAtY(line2, bottomY);
      if (x1 < x2) {
        left_line = line1;
        right_line = line2;
      } else {
        left_line = line2;
        right_line = line1;
      }
      leftFound = rightFound = true;
    } else if (found_line1) {
      int x1 = lineXAtY(line1, bottomY);
      if (x1 < width / 2) {
        left_line = line1;
        leftFound = true;
      } else {
        right_line = line1;
        rightFound = true;
      }
    }

    // 시각화: 검출된 차선 그리기
    if (leftFound)
      lane_visualize_line(visFrame, left_line, cv::Scalar(255, 0, 0), height);
    if (rightFound)
      lane_visualize_line(visFrame, right_line, cv::Scalar(0, 255, 0), height);

    // 차선 중앙 계산
    int lane_center_x = 0;
    if (leftFound && rightFound) {
      int left_x = lineXAtY(left_line, bottomY);
      int right_x = lineXAtY(right_line, bottomY);
      lane_center_x = (left_x + right_x) / 2;
    } else if (leftFound) {
      int left_x = lineXAtY(left_line, bottomY);
      lane_center_x = static_cast<int>(left_x + width / 2.2);
    } else if (rightFound) {
      int right_x = lineXAtY(right_line, bottomY);
      lane_center_x = static_cast<int>(right_x - width / 2.2);
    } else {
      lane_center_x = width / 2;
    }

    // Kalman 필터 적용: 측정된 차선 중심(lane_center_x)을 보정
    cv::Mat prediction = kalman_filter_.predict();
    cv::Mat measurement = (cv::Mat_<float>(1, 1) << static_cast<float>(lane_center_x));
    cv::Mat estimated = kalman_filter_.correct(measurement);
    float filtered_lane_center = estimated.at<float>(0);

    // 보정된 차선 중심을 시각화 (빨간 점)
    cv::circle(visFrame, cv::Point(static_cast<int>(filtered_lane_center), bottomY), 10, cv::Scalar(0, 0, 255), -1);

    // PID 제어: 필터 보정된 차선 중심을 기준으로 오차 계산
    double error = static_cast<double>(filtered_lane_center) - (width / 2.0);
    integral_ += error;
    double derivative = error - previous_error_;
    double steering = -(Kp_ * error + Ki_ * integral_ + Kd_ * derivative) / 3;
    previous_error_ = error;

    double steering_degree = steering * (180.0 / M_PI);
    double drive_speed = 0.0;
    if (fabs(steering_degree) <= 5.0)
      drive_speed = 1.5;
    else if (fabs(steering_degree) <= 8.0)
      drive_speed = 1.2;
    else if (fabs(steering_degree) <= 11.5)
      drive_speed = 1.0;
    else if (fabs(steering_degree) <= 15.0)
      drive_speed = 0.8;
    else
      drive_speed = 0.7;

    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = this->now();
    drive_msg.drive.steering_angle = steering;
    drive_msg.drive.speed = drive_speed;
    drive_pub_->publish(drive_msg);

    cv::imshow("binary", binary);
    cv::imshow("Lane Tracking", visFrame);
    cv::imshow("ROI Edges", edges);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  double Kp_, Ki_, Kd_;
  double previous_error_;
  double integral_;
  cv::KalmanFilter kalman_filter_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaneFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}
