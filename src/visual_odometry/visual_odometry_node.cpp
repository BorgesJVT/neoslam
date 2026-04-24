#include <iostream>
#include <memory>
#include <functional>
using namespace std;

// #include "utils/utils.h"
#include "visual_odometry.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

using namespace ratslam;

class VisualOdometryNode : public rclcpp::Node
{
public:
  VisualOdometryNode()
    : Node("visual_odometry_node")
  {
    // Declare all parameters with defaults
    this->declare_parameter("topic_root", "");
    this->declare_parameter("vtrans_image_x_min", 0);
    this->declare_parameter("vtrans_image_x_max", -1);
    this->declare_parameter("vtrans_image_y_min", 0);
    this->declare_parameter("vtrans_image_y_max", -1);
    this->declare_parameter("vrot_image_x_min", 0);
    this->declare_parameter("vrot_image_x_max", -1);
    this->declare_parameter("vrot_image_y_min", 0);
    this->declare_parameter("vrot_image_y_max", -1);
    this->declare_parameter("camera_fov_deg", 50.0);
    this->declare_parameter("camera_hz", 10.0);
    this->declare_parameter("vtrans_scaling", 100.0);
    this->declare_parameter("vtrans_max", 20.0);

    // Get all parameters
    std::string topic_root = this->get_parameter("topic_root").as_string();
    int vtrans_image_x_min = this->get_parameter("vtrans_image_x_min").as_int();
    int vtrans_image_x_max = this->get_parameter("vtrans_image_x_max").as_int();
    int vtrans_image_y_min = this->get_parameter("vtrans_image_y_min").as_int();
    int vtrans_image_y_max = this->get_parameter("vtrans_image_y_max").as_int();
    int vrot_image_x_min = this->get_parameter("vrot_image_x_min").as_int();
    int vrot_image_x_max = this->get_parameter("vrot_image_x_max").as_int();
    int vrot_image_y_min = this->get_parameter("vrot_image_y_min").as_int();
    int vrot_image_y_max = this->get_parameter("vrot_image_y_max").as_int();
    double camera_fov_deg = this->get_parameter("camera_fov_deg").as_double();
    double camera_hz = this->get_parameter("camera_hz").as_double();
    double vtrans_scaling = this->get_parameter("vtrans_scaling").as_double();
    double vtrans_max = this->get_parameter("vtrans_max").as_double();

    // Log all parameters for debugging
    RCLCPP_INFO(this->get_logger(), "VisualOdometry Parameters:");
    RCLCPP_INFO(this->get_logger(), "  topic_root: %s", topic_root.c_str());
    RCLCPP_INFO(this->get_logger(), "  vtrans_image_x_min: %d", vtrans_image_x_min);
    RCLCPP_INFO(this->get_logger(), "  vtrans_image_x_max: %d", vtrans_image_x_max);
    RCLCPP_INFO(this->get_logger(), "  vtrans_image_y_min: %d", vtrans_image_y_min);
    RCLCPP_INFO(this->get_logger(), "  vtrans_image_y_max: %d", vtrans_image_y_max);
    RCLCPP_INFO(this->get_logger(), "  vrot_image_x_min: %d", vrot_image_x_min);
    RCLCPP_INFO(this->get_logger(), "  vrot_image_x_max: %d", vrot_image_x_max);
    RCLCPP_INFO(this->get_logger(), "  vrot_image_y_min: %d", vrot_image_y_min);
    RCLCPP_INFO(this->get_logger(), "  vrot_image_y_max: %d", vrot_image_y_max);
    RCLCPP_INFO(this->get_logger(), "  camera_fov_deg: %f", camera_fov_deg);
    RCLCPP_INFO(this->get_logger(), "  camera_hz: %f", camera_hz);
    RCLCPP_INFO(this->get_logger(), "  vtrans_scaling: %f", vtrans_scaling);
    RCLCPP_INFO(this->get_logger(), "  vtrans_max: %f", vtrans_max);

    // Create VisualOdometry object with individual parameters
    vo = new ratslam::VisualOdometry(
      vtrans_image_x_min, vtrans_image_x_max,
      vtrans_image_y_min, vtrans_image_y_max,
      vrot_image_x_min, vrot_image_x_max,
      vrot_image_y_min, vrot_image_y_max,
      camera_fov_deg, camera_hz,
      vtrans_scaling, vtrans_max
    );

    pub_vo = this->create_publisher<nav_msgs::msg::Odometry>(
      topic_root + "/odom_VO", 10);
    
    sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_root + "/camera/image/compressed", 10,
      [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        this->image_callback(msg);
      });
    
    // To process image_raw
    sub_raw = this->create_subscription<sensor_msgs::msg::Image>(
      topic_root + "/camera/image", 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->image_callback(msg);
      });

    RCLCPP_INFO(this->get_logger(), "VisualOdometry node initialized");
    // RCLCPP_INFO(this->get_logger(), "VisualOdometry node initialized, subscribing to %s/camera/image/compressed", topic_root.c_str());
  }

  ~VisualOdometryNode()
  {
    if (vo != nullptr)
    {
      delete vo;
      vo = nullptr;
    }
  }

private:
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image_msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "VO:image_compressed_callback");

    // Decode compressed image
    cv::Mat image;
    try
    {
      image = cv::imdecode(cv::Mat(image_msg->data), cv::IMREAD_COLOR);
      if (image.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
        return;
      }
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
      return;
    }

    nav_msgs::msg::Odometry odom_output;

    vo->on_image(image.data, false, image.cols, image.rows, 
                 &odom_output.twist.twist.linear.x, 
                 &odom_output.twist.twist.angular.z);

    odom_output.header.stamp = image_msg->header.stamp;

    pub_vo->publish(odom_output);
  }

  // callback for raw image topic
  void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "VO:image_raw_callback");

    // Convert ROS Image to OpenCV Mat using cv_bridge
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      if (cv_ptr->image.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert image");
        return;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    nav_msgs::msg::Odometry odom_output;

    vo->on_image(cv_ptr->image.data, false, cv_ptr->image.cols, cv_ptr->image.rows, 
                 &odom_output.twist.twist.linear.x, 
                 &odom_output.twist.twist.angular.z);

    odom_output.header.stamp = image_msg->header.stamp;

    pub_vo->publish(odom_output);
  }

  ratslam::VisualOdometry* vo = nullptr;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_vo;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_raw;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::cout << argv[0] << " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath" << std::endl;
  std::cout << "RatSLAM algorithm by Michael Milford and Gordon Wyeth" << std::endl;
  std::cout << "Distributed under the GNU GPL v3, see the included license file." << std::endl;

  auto node = std::make_shared<VisualOdometryNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
