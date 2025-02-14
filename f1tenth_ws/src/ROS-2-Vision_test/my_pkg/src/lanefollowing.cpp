#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
// ximgproc 헤더 (opencv_contrib 모듈 필요)
#include <opencv2/ximgproc.hpp>
#include <vector>
#include <numeric>
#include <cstdlib>
#include <ctime>

class LaneFollowerRansacNode : public rclcpp::Node {
public:
  LaneFollowerRansacNode()
  : Node("lane_follower_ransac_node"), previous_error_(0.0), integral_(0.0)
  {
    // 랜덤 시드 초기화
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    // PID 파라미터 선언 및 불러오기
    this->declare_parameter<double>("Kp", 0.1);
    this->declare_parameter<double>("Ki", 0.00);
    this->declare_parameter<double>("Kd", 0.00);
    this->get_parameter("Kp", Kp_);
    this->get_parameter("Ki", Ki_);
    this->get_parameter("Kd", Kd_);

    // 카메라 이미지 구독자 생성
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 10,
      std::bind(&LaneFollowerRansacNode::image_callback, this, std::placeholders::_1));

    // 구동 명령 퍼블리셔 생성
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", 10);

    // 결과를 시각화할 윈도우 생성
    cv::namedWindow("Lane Tracking", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("ROI Edges", cv::WINDOW_AUTOSIZE);

    RCLCPP_INFO(this->get_logger(), "LaneFollowerRansacNode has been started.");
  }

private:
  // RANSAC 알고리즘을 통한 선형 모델 피팅
  // 입력: points - 후보 점들, bestLine - (a, b, c) 형태의 선 (a*x + b*y + c = 0)
  // iterations: 반복 횟수, threshold: inlier 판별 임계값, minInliers: 최소 inlier 수
  bool ransacLineFit(const std::vector<cv::Point>& points, cv::Vec3f &bestLine,
                     int iterations = 150, double threshold = 3.0, int minInliers = 25)
  {
    if(points.size() < 2)
      return false;
    
    int bestInlierCount = 0;
    cv::Vec3f bestCandidate;
    std::vector<cv::Point> bestInliers;

    for (int i = 0; i < iterations; i++) {
      // 두 점을 무작위로 선택
      int idx1 = std::rand() % points.size();
      int idx2 = std::rand() % points.size();
      if(idx1 == idx2)
        continue;
      cv::Point p1 = points[idx1];
      cv::Point p2 = points[idx2];

      // 두 점을 지나는 선: a*x + b*y + c = 0
      float a = p2.y - p1.y;
      float b = p1.x - p2.x;
      float c = p2.x * p1.y - p1.x * p2.y;
      float norm = std::sqrt(a * a + b * b);
      if(norm < 1e-6)
        continue;
      a /= norm; b /= norm; c /= norm; // 정규화

      // inlier 판별
      std::vector<cv::Point> inliers;
      for(const auto &p : points) {
        float dist = std::fabs(a * p.x + b * p.y + c);
        if(dist < threshold) {
          inliers.push_back(p);
        }
      }
      if(inliers.size() > bestInlierCount) {
        bestInlierCount = static_cast<int>(inliers.size());
        bestCandidate = cv::Vec3f(a, b, c);
        bestInliers = inliers;
      }
    }
    if(bestInlierCount < minInliers)
      return false;

    // inlier 점들을 사용해 cv::fitLine()으로 선형 모델 재피팅
    cv::Vec4f lineFit;
    cv::fitLine(bestInliers, lineFit, cv::DIST_L2, 0, 0.01, 0.01);
    float vx = lineFit[0], vy = lineFit[1], x0 = lineFit[2], y0 = lineFit[3];
    // (vx, vy)는 선의 방향벡터, (x0, y0)는 선 위의 한 점
    // 선의 법선 벡터는 (-vy, vx)가 됨.
    float a = -vy;
    float b = vx;
    float c = vy * x0 - vx * y0;
    float norm2 = std::sqrt(a*a + b*b);
    if(norm2 < 1e-6)
      return false;
    a /= norm2; b /= norm2; c /= norm2;
    bestLine = cv::Vec3f(a, b, c);
    return true;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // ROS 이미지 메시지를 OpenCV 이미지로 변환
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

    // 1. 전처리: HSV 변환 및 임계값 적용 (밝은 영역만 추출)
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    // 밝은 영역 범위를 조정하여 불필요한 잡음을 줄임
    cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(180, 30, 255), mask);

    // 2. 가우시안 블러를 통해 노이즈 제거
    cv::Mat blurred;
    cv::GaussianBlur(mask, blurred, cv::Size(7, 7), 0);

    // 3. 모폴로지 연산: 침식과 팽창으로 작은 노이즈 제거
    cv::Mat morph;
    cv::erode(blurred, morph, cv::Mat(), cv::Point(-1,-1), 4);
    cv::dilate(morph, morph, cv::Mat(), cv::Point(-1,-1), 2);

    // 4. Canny 에지 검출 (매개변수 조정)
    cv::Mat edges;
    cv::Canny(morph, edges, 80, 200);

    // 5. Thinning(스켈레톤) 처리: 두꺼운 에지를 단일 픽셀 두께로 줄임
    cv::Mat thinEdges;
    cv::ximgproc::thinning(edges, thinEdges, cv::ximgproc::THINNING_ZHANGSUEN);

    // 6. ROI 설정: 이미지 아래쪽 절반 영역 사용
    cv::Rect roi_rect(0, height/2, width, height/2);
    cv::Mat roi = thinEdges(roi_rect);

    // 7. ROI 내 non-zero(에지) 픽셀 좌표 추출 (전체 이미지 좌표로 보정)
    std::vector<cv::Point> roiPoints;
    cv::findNonZero(roi, roiPoints);
    for(auto &pt : roiPoints) {
      pt.y += height/2;
    }

    // 8. 후보 점들을 좌측과 우측으로 분할 (이미지 중앙 기준)
    std::vector<cv::Point> leftPoints, rightPoints;
    for(const auto &pt : roiPoints) {
      if(pt.x < width/2)
        leftPoints.push_back(pt);
      else
        rightPoints.push_back(pt);
    }

    // 9. 각 후보 점 집합에 대해 RANSAC 선 피팅 수행
    cv::Vec3f leftLine, rightLine;
    bool leftFound = ransacLineFit(leftPoints, leftLine);
    bool rightFound = ransacLineFit(rightPoints, rightLine);

    // 10. 시각화를 위해 원본 이미지를 복사하고 후보점 표시
    cv::Mat visFrame = frame.clone();
    for(const auto &pt : leftPoints) {
      cv::circle(visFrame, pt, 2, cv::Scalar(255, 0, 0), -1);
    }
    for(const auto &pt : rightPoints) {
      cv::circle(visFrame, pt, 2, cv::Scalar(0, 255, 0), -1);
    }

    // ROI의 상단과 하단 y 좌표
    int y_top = height/2;
    int y_bottom = height - 1;
    int left_x_top = 0, left_x_bottom = 0;
    int right_x_top = 0, right_x_bottom = 0;
    
    // 11. RANSAC 결과로 피팅된 선 그리기
    if(leftFound && std::fabs(leftLine[0]) > 1e-6) {
      // 선 방정식: a*x + b*y + c = 0 → x = (-b*y - c) / a
      left_x_top = static_cast<int>((-leftLine[1] * y_top - leftLine[2]) / leftLine[0]);
      left_x_bottom = static_cast<int>((-leftLine[1] * y_bottom - leftLine[2]) / leftLine[0]);
      cv::line(visFrame, cv::Point(left_x_top, y_top), cv::Point(left_x_bottom, y_bottom),
               cv::Scalar(255, 0, 255), 3);
    }
    if(rightFound && std::fabs(rightLine[0]) > 1e-6) {
      right_x_top = static_cast<int>((-rightLine[1] * y_top - rightLine[2]) / rightLine[0]);
      right_x_bottom = static_cast<int>((-rightLine[1] * y_bottom - rightLine[2]) / rightLine[0]);
      cv::line(visFrame, cv::Point(right_x_top, y_top), cv::Point(right_x_bottom, y_bottom),
               cv::Scalar(0, 255, 255), 3);
    }

    // 12. 양쪽 선이 검출되면 차선 중앙 계산 (하단 좌표 기준)
    int lane_center_x = width/2;
    if(leftFound && rightFound) {
      lane_center_x = (left_x_bottom + right_x_bottom) / 2;
      cv::circle(visFrame, cv::Point(left_x_bottom, y_bottom), 5, cv::Scalar(255, 0, 255), -1);
      cv::circle(visFrame, cv::Point(right_x_bottom, y_bottom), 5, cv::Scalar(0, 255, 255), -1);
      cv::line(visFrame, cv::Point(left_x_bottom, y_bottom), cv::Point(right_x_bottom, y_bottom),
               cv::Scalar(0, 0, 255), 2);
    }

    // 13. PID 제어: 이미지 중앙과 차선 중앙의 오차 계산 후 제어
    double error = lane_center_x - (width/2);
    integral_ += error;
    double derivative = error - previous_error_;
    double steering = -(Kp_ * error + Ki_ * integral_ + Kd_ * derivative);
    previous_error_ = error;

    // 차량 제어 명령 퍼블리시
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = this->now();
    drive_msg.drive.steering_angle = steering;
    drive_msg.drive.speed = 0.5;
    drive_pub_->publish(drive_msg);

    // 14. 시각화: 차선 중앙 및 스티어링 값 표시
    cv::circle(visFrame, cv::Point(lane_center_x, y_bottom), 10, cv::Scalar(0, 0, 255), -1);
    std::string text = "Steering: " + std::to_string(steering);
    cv::putText(visFrame, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                cv::Scalar(255, 0, 0), 2);

    cv::imshow("Lane Tracking", visFrame);
    cv::imshow("ROI Edges", roi);
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
  auto node = std::make_shared<LaneFollowerRansacNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}
