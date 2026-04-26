#include <iostream>
#include <memory>
#include <functional>
using namespace std;

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomFusionNode : public rclcpp::Node
{
public:
  OdomFusionNode()
    : Node("odom_fusion_node")
  {
    // Declare all parameters with defaults
    this->declare_parameter("topic_root", "ROSI");

    // Get all parameters
    std::string topic_root = this->get_parameter("topic_root").as_string();

    // Publishers
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>(
      topic_root + "/odom", 10);
    
    rclcpp::QoS flexible_qos(10);
    flexible_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    // Aceita BEST_EFFORT
    // Mas também funciona com RELIABLE devido à compatibilidade

    // Subscribers
    sub_steering = this->create_subscription<nav_msgs::msg::Odometry>(
      topic_root + "/steering_model/odom", flexible_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->steering_callback(msg);
      });

    sub_vo = this->create_subscription<nav_msgs::msg::Odometry>(
    //   topic_root + "/odom_VO", 10,
         "/T265/pose/sample", flexible_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->vo_callback(msg);
      });

    RCLCPP_INFO(this->get_logger(), "OdomFusion node initialized");
  }

  ~OdomFusionNode()
  {
  }

private:
  void steering_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_steering_ = msg;
    publish_fused_odom();
  }

  void vo_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_vo_ = msg;
    publish_fused_odom();
  }

  void publish_fused_odom()
  {
    // Aguardar receber mensagem de ambos os tópicos
    if (!last_steering_ || !last_vo_)
      return;

    nav_msgs::msg::Odometry odom_output;

    // Timestamp: usar o mais recente
    if (last_steering_->header.stamp.sec > last_vo_->header.stamp.sec ||
        (last_steering_->header.stamp.sec == last_vo_->header.stamp.sec &&
         last_steering_->header.stamp.nanosec > last_vo_->header.stamp.nanosec))
    {
      odom_output.header.stamp = last_steering_->header.stamp;
    }
    else
    {
      odom_output.header.stamp = last_vo_->header.stamp;
    }

    odom_output.header.frame_id = "world";
    odom_output.child_frame_id = "base_link";

    // Twist linear vem de steering_model
    odom_output.twist.twist.linear.x = last_steering_->twist.twist.linear.x;
    odom_output.twist.twist.linear.y = last_steering_->twist.twist.linear.y;
    odom_output.twist.twist.linear.z = last_steering_->twist.twist.linear.z;

    // Twist angular vem de odom_VO
    odom_output.twist.twist.angular.x = last_vo_->twist.twist.angular.x;
    odom_output.twist.twist.angular.y = last_vo_->twist.twist.angular.y;
    odom_output.twist.twist.angular.z = last_vo_->twist.twist.angular.z;

    // Pose: usar a mais recente (do VO que tem pose completa)
    rclcpp::Time vo_time(last_vo_->header.stamp);
    rclcpp::Time steering_time(last_steering_->header.stamp);
    if (vo_time > steering_time)
    {
      odom_output.pose = last_vo_->pose;
    }
    else
    {
      odom_output.pose = last_steering_->pose;
    }

    // Publicar odometria fundida
    pub_odom->publish(odom_output);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_steering;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vo;

  nav_msgs::msg::Odometry::SharedPtr last_steering_;
  nav_msgs::msg::Odometry::SharedPtr last_vo_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::cout << argv[0] << " - NeoSLAM OdomFusion Node" << std::endl;
  std::cout << "Distributed under the GNU GPL v3, see the included license file." << std::endl;

  auto node = std::make_shared<OdomFusionNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}