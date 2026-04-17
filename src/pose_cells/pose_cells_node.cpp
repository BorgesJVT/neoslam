#include <iostream>
#include <memory>
#include <functional>
using namespace std;

#include <rclcpp/rclcpp.hpp>

#include "posecell_network.h"
#include <topological_msgs/msg/topological_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <topological_msgs/msg/view_template.hpp>


#if HAVE_IRRLICHT
#include "posecell_scene.h"
PosecellScene *pcs;
bool use_graphics;
#endif

class PoseCellNode : public rclcpp::Node
{
public:
  PoseCellNode()
    : Node("pose_cells_node")
  {
    // Declare and get parameters
    this->declare_parameter("topic_root", "");
    this->declare_parameter("pc_dim_xy", 21);
    this->declare_parameter("pc_dim_th", 36);
    this->declare_parameter("pc_w_e_dim", 7);
    this->declare_parameter("pc_w_i_dim", 5);
    this->declare_parameter("pc_w_e_var", 1);
    this->declare_parameter("pc_w_i_var", 2);
    this->declare_parameter("pc_global_inhib", 0.00002);
    this->declare_parameter("vt_active_decay", 1.0);
    this->declare_parameter("pc_vt_inject_energy", 0.15);
    this->declare_parameter("pc_cell_x_size", 1.0);
    this->declare_parameter("exp_delta_pc_threshold", 2.0);
    this->declare_parameter("pc_vt_restore", 0.05);
    this->declare_parameter("enable", true);
    this->declare_parameter("posecells_size", 250);
    this->declare_parameter("media_path", "");
    this->declare_parameter("image_file", "");
    
    std::string topic_root = this->get_parameter("topic_root").as_string();
    
    // Log all parameters for debugging
    RCLCPP_INFO(this->get_logger(), "PoseCell Parameters:");
    RCLCPP_INFO(this->get_logger(), "  topic_root: %s", topic_root.c_str());
    RCLCPP_INFO(this->get_logger(), "  pc_dim_xy: %ld", this->get_parameter("pc_dim_xy").as_int());
    RCLCPP_INFO(this->get_logger(), "  pc_dim_th: %ld", this->get_parameter("pc_dim_th").as_int());
    RCLCPP_INFO(this->get_logger(), "  vt_active_decay: %f", this->get_parameter("vt_active_decay").as_double());
    RCLCPP_INFO(this->get_logger(), "  pc_vt_inject_energy: %f", this->get_parameter("pc_vt_inject_energy").as_double());
    RCLCPP_INFO(this->get_logger(), "  pc_cell_x_size: %f", this->get_parameter("pc_cell_x_size").as_double());
    RCLCPP_INFO(this->get_logger(), "  exp_delta_pc_threshold: %f", this->get_parameter("exp_delta_pc_threshold").as_double());
    RCLCPP_INFO(this->get_logger(), "  pc_vt_restore: %f", this->get_parameter("pc_vt_restore").as_double());
    
    // Create PosecellNetwork with parameters
    pc = new PosecellNetwork(
      this->get_parameter("pc_dim_xy").as_int(),
      this->get_parameter("pc_dim_th").as_int(),
      this->get_parameter("pc_w_e_dim").as_int(),
      this->get_parameter("pc_w_i_dim").as_int(),
      this->get_parameter("pc_w_e_var").as_int(),
      this->get_parameter("pc_w_i_var").as_int(),
      this->get_parameter("pc_global_inhib").as_double(),
      this->get_parameter("vt_active_decay").as_double(),
      this->get_parameter("pc_vt_inject_energy").as_double(),
      this->get_parameter("pc_cell_x_size").as_double(),
      this->get_parameter("exp_delta_pc_threshold").as_double(),
      this->get_parameter("pc_vt_restore").as_double()
    );
    
    pub_pc = this->create_publisher<topological_msgs::msg::TopologicalAction>(
      topic_root + "/PoseCell/TopologicalAction", 10);

    rclcpp::QoS flexible_qos(10);
    flexible_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    // Aceita BEST_EFFORT
    // Mas também funciona com RELIABLE devido à compatibilidade
    
    sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
      topic_root + "/odom",
      flexible_qos,
      std::bind(&PoseCellNode::odo_callback, this, std::placeholders::_1));
    
    sub_template = this->create_subscription<topological_msgs::msg::ViewTemplate>(
      topic_root + "/LocalView/Template", 10,
      std::bind(&PoseCellNode::template_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "PoseCell node initialized");
    
#ifdef HAVE_IRRLICHT
    use_graphics = this->get_parameter("enable").as_bool();
    if (use_graphics)
    {
      pcs = new PosecellScene(
        this->get_parameter("posecells_size").as_int(),
        this->get_parameter("media_path").as_string(),
        this->get_parameter("image_file").as_string(),
        pc
      );
    }
#endif
  }
  
  ~PoseCellNode()
  {
    if (pc != nullptr)
      delete pc;
#ifdef HAVE_IRRLICHT
    if (pcs != nullptr)
      delete pcs;
#endif
  }

private:
  void odo_callback(const nav_msgs::msg::Odometry::SharedPtr odo)
  {
    RCLCPP_DEBUG(this->get_logger(), "PC:odo_callback v=%f r=%f", 
                 odo->twist.twist.linear.x, odo->twist.twist.angular.z);

    if (prev_time.seconds() > 0)
    {
      double time_diff = (rclcpp::Time(odo->header.stamp) - prev_time).seconds();

      pc_output.src_id = pc->get_current_exp_id();
      pc->on_odo(odo->twist.twist.linear.x, odo->twist.twist.angular.z, time_diff);
      pc_output.action = pc->get_action();
      
      if (pc_output.action != PosecellNetwork::NO_ACTION)
      {
        pc_output.header.stamp = odo->header.stamp;
        pc_output.dest_id = pc->get_current_exp_id();
        pc_output.relative_rad = pc->get_relative_rad();
        pub_pc->publish(pc_output);
        
        RCLCPP_DEBUG(this->get_logger(), "PC:action_publish action=%d src=%d dest=%d",
                     pc_output.action, pc_output.src_id, pc_output.dest_id);
      }

#ifdef HAVE_IRRLICHT
      if (use_graphics)
      {
        pcs->update_scene();
        pcs->draw_all();
      }
#endif
    }
    prev_time = rclcpp::Time(odo->header.stamp);
  }

  void template_callback(const topological_msgs::msg::ViewTemplate::SharedPtr vt)
  {
    RCLCPP_DEBUG(this->get_logger(), "PC:vt_callback id=%d rad=%f", 
                 vt->current_id, vt->relative_rad);

    pc->on_view_template(vt->current_id, vt->relative_rad);

#ifdef HAVE_IRRLICHT
    if (use_graphics)
    {
      pcs->update_scene();
      pcs->draw_all();
    }
#endif
  }

  PosecellNetwork* pc = nullptr;
  rclcpp::Publisher<topological_msgs::msg::TopologicalAction>::SharedPtr pub_pc;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;
  rclcpp::Subscription<topological_msgs::msg::ViewTemplate>::SharedPtr sub_template;
  rclcpp::Time prev_time{0, 0, RCL_ROS_TIME};
  topological_msgs::msg::TopologicalAction pc_output;

#ifdef HAVE_IRRLICHT
  PosecellScene *pcs = nullptr;
  bool use_graphics;
#endif
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseCellNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
