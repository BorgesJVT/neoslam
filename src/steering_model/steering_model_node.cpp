#include <iostream>
#include <memory>
#include <functional>
using namespace std;

#include <rclcpp/rclcpp.hpp>
#include <rosi.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace rosi_controller;

class SteeringModelNode : public rclcpp::Node, public Rosi
{
public:
  SteeringModelNode()
    : Node("steering_model_node")
  {
    // Declare all parameters with defaults
    this->declare_parameter("topic_root", "ROSI");
  
    // Get all parameters
    std::string topic_root = this->get_parameter("topic_root").as_string();

    if (setBodyWidthWheels(this->declare_parameter("body_width_wheels", -1.0)) < 0)
    {   
        rclcpp::shutdown();
        throw std::runtime_error("[ ERROR] Could not find parameter body_width_wheels.");
    }

    if (setBodyWidthTracks(this->declare_parameter("body_width_tracks", -1.0)) < 0)
    {   
        rclcpp::shutdown();
        throw std::runtime_error("[ ERROR] Could not find parameter body_width_tracks.");
    }

    if (setWheelsRadius(this->declare_parameter("wheels_radius", -1.0)) < 0)
    {   
        rclcpp::shutdown();
        throw std::runtime_error("[ ERROR] Could not find parameter wheels_radius.");
    }

    if (setTracksRadius(this->declare_parameter("tracks_radius", -1.0)) < 0)
    {   
        rclcpp::shutdown();
        throw std::runtime_error("[ ERROR] Could not find parameter tracks_radius.");
    }

    if (setMass(this->declare_parameter("mass", -1.0)) < 0)
    {   
        rclcpp::shutdown();
        throw std::runtime_error("[ ERROR] Could not find parameter mass.");
    }

    if (setIzz(this->declare_parameter("izz", -1.0)) < 0)
    {   
        rclcpp::shutdown();
        throw std::runtime_error("[ ERROR] Could not find parameter izz.");
    }

    if (setWheelCurrent2Torque(this->declare_parameter("wheel_current_2_torque", -1.0)) < 0)
    {   
        rclcpp::shutdown();
        throw std::runtime_error("[ ERROR] Could not find parameter wheel_current_2_torque.");
    }

    if (setArmCurrent2Torque(this->declare_parameter("arm_current_2_torque", -1.0)) < 0)
    {   
        rclcpp::shutdown();
        throw std::runtime_error("[ ERROR] Could not find parameter arm_current_2_torque.");
    }

    // Create publisher for odometry
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>(
      topic_root + "/steering_model/odom", 10);
    
    
    // Create subscriber for joint states
    sub_joint_states = this->create_subscription<sensor_msgs::msg::JointState>(
    "/rosi/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->joint_state_callback(msg);
      });

    RCLCPP_INFO(this->get_logger(), "SteeringModel node initialized");
  }

  ~SteeringModelNode()
  {
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "SteeringModel: joint_state_callback");

    // Armazenar o joint state no objeto Rosi
    *getCurrentJointStatePtr() = *joint_state_msg;

    // Publish Odometry information: pose from NOT DEFINED and twist from wheels velocities
    nav_msgs::msg::Odometry odom_output;
    
    // Set header timestamp from joint state message
    odom_output.header.stamp = joint_state_msg->header.stamp;
    odom_output.header.frame_id = "base_link";
  
    getOdomVelocity(getCurrentJointStatePtr()->velocity, odom_output.twist.twist.linear.x, odom_output.twist.twist.angular.z);
    // odom_output.header.stamp = this->now();
    odom_output.header.stamp = joint_state_msg->header.stamp;
    odom_output.header.frame_id = "world";
    odom_output.child_frame_id = "base_link";

    // set odometry covariance matrix
    std::vector<double> odom_covariance_local(36);
    double covariance[36]={0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001};
    odom_covariance_local.assign(covariance, covariance+36);
    std::copy(odom_covariance_local.begin(), odom_covariance_local.end(), odom_output.pose.covariance.begin());
    std::copy(odom_covariance_local.begin(), odom_covariance_local.end(), odom_output.twist.covariance.begin());

    // Publish odometry
    pub_odom->publish(odom_output);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::cout << argv[0] << " - NeoSLAM SteeringModel Node" << std::endl;
  std::cout << "Distributed under the GNU GPL v3, see the included license file." << std::endl;

  auto node = std::make_shared<SteeringModelNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
