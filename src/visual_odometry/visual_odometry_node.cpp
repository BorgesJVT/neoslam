#include <iostream>
#include <memory>
#include <functional>
using namespace std;

#include "ratslam/visual_odometry.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

using namespace ratslam;

/**
 * @brief VisualOdometryNode - Vision-based motion estimation from camera images
 * 
 * This ROS2 node estimates robot motion (linear and angular velocity) by analyzing
 * sequential camera images. It uses optical flow techniques on specific image regions
 * to compute visual odometry: translation is estimated from the bottom portion of the
 * image (ground plane), while rotation is estimated from the full image. This provides
 * an alternative or complement to wheel odometry, especially useful for robots with
 * poor wheel encoders or on slippery terrain.
 * 
 * Subscriptions:
 *   - <topic_root>/camera/image/compressed: Receives compressed camera images (JPEG)
 * 
 * Publications:
 *   - <topic_root>/odom: Publishes estimated odometry (linear and angular velocity)
 * 
 * Parameters:
 *   - topic_root: Base topic namespace for all subscriptions and publications
 *   - vtrans_image_x_min: Min X coordinate for translation estimation region (default: 0)
 *   - vtrans_image_x_max: Max X coordinate for translation estimation region (-1 = image width)
 *   - vtrans_image_y_min: Min Y coordinate for translation estimation region (default: 0)
 *   - vtrans_image_y_max: Max Y coordinate for translation estimation region (-1 = image height)
 *   - vrot_image_x_min: Min X coordinate for rotation estimation region (default: 0)
 *   - vrot_image_x_max: Max X coordinate for rotation estimation region (-1 = image width)
 *   - vrot_image_y_min: Min Y coordinate for rotation estimation region (default: 0)
 *   - vrot_image_y_max: Max Y coordinate for rotation estimation region (-1 = image height)
 *   - camera_fov_deg: Camera horizontal field of view in degrees (default: 50.0)
 *   - camera_hz: Camera frame rate in Hz for velocity scaling (default: 10.0)
 *   - vtrans_scaling: Scaling factor for translation velocity (default: 100.0)
 *   - vtrans_max: Maximum translation velocity in m/s (default: 20.0)
 */
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
      topic_root + "/odom", 10);
    
    sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_root + "/camera/image/compressed", 10,
      std::bind(&VisualOdometryNode::image_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "VisualOdometry node initialized, subscribing to %s/camera/image/compressed", topic_root.c_str());
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
    RCLCPP_DEBUG(this->get_logger(), "VO:image_callback");

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

  ratslam::VisualOdometry* vo = nullptr;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_vo;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub;
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
