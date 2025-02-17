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
#include <cmath>

class LaneFollowerPolyNode : public rclcpp::Node {
public:
  LaneFollowerPolyNode()
  : Node("lane_follower_node"), previous_error_(0.0), integral_(0.0)
  {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    // PID 파라미터 선언 및 불러오기
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

    cv::namedWindow("Lane Tracking", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("ROI Edges", cv::WINDOW_AUTOSIZE);

    RCLCPP_INFO(this->get_logger(), "LaneFollowerNode has been started.");
  }

private:


  // polynomal fitting function
  std::vector<double> polyFit(const std::vector<cv::Point>& points, int degree = 2) {
  
    int n = points.size();
    std::vector<double> coeff;
    if (n == 0) return coeff;
    
    cv::Mat X(n, degree + 1, CV_64F);
    cv::Mat Y(n, 1, CV_64F);
    for (int i = 0; i < n; i++) {
      double y = points[i].y;
      Y.at<double>(i, 0) = points[i].x;  // x = f(y)
      for (int j = 0; j < degree + 1; j++) {
        X.at<double>(i, j) = std::pow(y, degree - j);
      }
    }
    cv::Mat coeffMat;
    cv::solve(X, Y, coeffMat, cv::DECOMP_SVD);
    for (int i = 0; i < coeffMat.rows; i++) {
      coeff.push_back(coeffMat.at<double>(i, 0));
    }
    return coeff;  // coeff[0]=A, coeff[1]=B, coeff[2]=C
  }


  // visualizing poly function
  void lane_visualize(cv::Mat &img, const std::vector<double>& poly, cv::Scalar color) {
  
    if(poly.empty()) return;
    int degree = poly.size() - 1; // 보통 2
    for (int y = img.rows/2; y < img.rows; y++) {
      double x = 0;
      for (int j = 0; j < poly.size(); j++) {
        x += poly[j] * std::pow(y, degree - j);
      }
      cv::circle(img, cv::Point(static_cast<int>(x), y), 2, color, -1);
    }
  }

  // 하단에서 좌우 다항식 곡선을 통해 계산된 x좌표로 차선 중앙을 구함
  int LaneCenter(const std::vector<double>& leftPoly, const std::vector<double>& rightPoly, int y) {
  
    double left_x = 0, right_x = 0;
    int degree;
    if (!leftPoly.empty()) {
      degree = leftPoly.size() - 1;
      for (int j = 0; j < leftPoly.size(); j++) {
        left_x += leftPoly[j] * std::pow(y, degree - j);
      }
    }
    if (!rightPoly.empty()) {
      degree = rightPoly.size() - 1;
      for (int j = 0; j < rightPoly.size(); j++) {
        right_x += rightPoly[j] * std::pow(y, degree - j);
      }
    }
    if (!leftPoly.empty() && !rightPoly.empty())
      return static_cast<int>((left_x + right_x) / 2);
    if (!leftPoly.empty())
      return static_cast<int>(left_x + 200);
    if (!rightPoly.empty())
      return static_cast<int>(right_x - 200);
    return -1;
  }

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

    // HSV filter
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    
    // white line
    //cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(180, 30, 255), mask);
    
    // yellow line
    cv::inRange(hsv, cv::Scalar(20, 50, 120), cv::Scalar(70, 255, 255), mask);


    // gaussianblur
    cv::Mat blurred;
    cv::GaussianBlur(mask, blurred, cv::Size(7, 7), 0);

    // morphology (noise)
    cv::Mat morph;
    cv::erode(blurred, morph, cv::Mat(), cv::Point(-1,-1), 2);
    cv::dilate(morph, morph, cv::Mat(), cv::Point(-1,-1), 1);

    // Canny edge
    cv::Mat edges;
    cv::Canny(morph, edges, 80, 200);

    // Thinning
    cv::Mat thinEdges;
    cv::ximgproc::thinning(edges, thinEdges, cv::ximgproc::THINNING_ZHANGSUEN);

    // ROI setting
    cv::Rect roi_rect(0, height/2, width, height/2);
    cv::Mat roi = thinEdges(roi_rect);

    // edge
    std::vector<cv::Point> roiPoints;
    cv::findNonZero(roi, roiPoints);
    for(auto &pt : roiPoints) {
      pt.y += height/2;
    }

    // seperate left / right lane
    std::vector<cv::Point> leftPoints, rightPoints;
    for(const auto &pt : roiPoints) {
      if(pt.x < width/2)
        leftPoints.push_back(pt);
      else
        rightPoints.push_back(pt);
    }

    // poly fitting --> 2 order
    std::vector<double> leftPoly = polyFit(leftPoints, 3);
    std::vector<double> rightPoly = polyFit(rightPoints, 3);

    // Visualizing lane
    cv::Mat visFrame = frame.clone();
    lane_visualize(visFrame, leftPoly, cv::Scalar(255, 0, 0));   // left lane --> blue
    lane_visualize(visFrame, rightPoly, cv::Scalar(0, 255, 0));   // right lane --> green

    // lane center visualize
    int lane_center_x = LaneCenter(leftPoly, rightPoly, height - 1);
    cv::circle(visFrame, cv::Point(lane_center_x, height - 1), 10, cv::Scalar(0, 0, 255), -1);

    // pid control & drive topic publish
    double error = lane_center_x - (width/2);
    integral_ += error;
    double derivative = error - previous_error_;
    double steering = -(Kp_ * error + Ki_ * integral_ + Kd_ * derivative);
    previous_error_ = error;

    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = this->now();
    drive_msg.drive.steering_angle = steering / 3;
    drive_msg.drive.speed = 0.5;
    drive_pub_->publish(drive_msg);

    // visualize
    //cv::imshow("HSV", mask);
    //cv::imshow("Gauusianed", morph);
    //cv::imshow("thinned", thinEdges);
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
  auto node = std::make_shared<LaneFollowerPolyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}
