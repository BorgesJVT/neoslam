#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <topological_msgs/msg/sdr_stamped.hpp>
#include <topological_msgs/msg/view_template.hpp>
#include <roaring/roaring.hh>
#include <memory>
#include <iostream>
#include <chrono>
#include <vector>

#include "local_view_match_with_intervals.hpp"

/**
 * @brief SpatialViewCellsNode - Visual place recognition and loop closure detection
 * 
 * This ROS2 node processes Sparse Distributed Representations (SDR) from visual features
 * to recognize unique places (visual templates) and detect loop closures. It uses an
 * interval-based matching algorithm with Roaring Bitmaps for efficient similarity 
 * computation and memory usage.
 * 
 * Subscriptions:
 *   - <topic_root>/sdr: Receives SDR representations from visual features
 * 
 * Publications:
 *   - <topic_root>/LocalView/Template: Publishes recognized visual template IDs
 * 
 * Parameters:
 *   - topic_root: Base topic namespace for all subscriptions and publications
 *   - theta_alpha: Similarity threshold for extending intervals (default: 384 active bits)
 *   - theta_rho: Maximum duration of an interval in frames (default: 3 frames)
 *   - score_interval: Similarity threshold for loop closure detection (default: 470 matching bits)
 *   - exclude_recent_intervals: Number of recent intervals to exclude from loop closure detection (default: 3)
 */
class SpatialViewCellsNode : public rclcpp::Node {
public:
    explicit SpatialViewCellsNode()
        : Node("spatial_view_cells_node"), counter_(0) {
        
        // Declare and get parameters
        this->declare_parameter("topic_root", "");
        this->declare_parameter<int>("theta_alpha", 384);
        this->declare_parameter<int>("theta_rho", 3);
        this->declare_parameter<int>("score_interval", 470);
        this->declare_parameter<int>("exclude_recent_intervals", 3);
        
        std::string topic_root = this->get_parameter("topic_root").as_string();
        int theta_alpha = this->get_parameter("theta_alpha").as_int();
        int theta_rho = this->get_parameter("theta_rho").as_int();
        int score_interval = this->get_parameter("score_interval").as_int();
        int exclude_recent_intervals = this->get_parameter("exclude_recent_intervals").as_int();
        
        RCLCPP_INFO(this->get_logger(), "SpatialViewCellsNode Parameters:");
        RCLCPP_INFO(this->get_logger(), "  topic_root: %s", topic_root.c_str());
        RCLCPP_INFO(this->get_logger(), "  theta_alpha: %d", theta_alpha);
        RCLCPP_INFO(this->get_logger(), "  theta_rho: %d", theta_rho);
        RCLCPP_INFO(this->get_logger(), "  score_interval: %d", score_interval);
        RCLCPP_INFO(this->get_logger(), "  exclude_recent_intervals: %d", exclude_recent_intervals);
        
        // Initialize LocalViewMatchWithIntervals
        lv_ = std::make_unique<LocalViewMatchWithIntervals>(
            theta_alpha, theta_rho, score_interval, exclude_recent_intervals
        );
        
        // Create publisher for view templates
        pub_vt_ = this->create_publisher<topological_msgs::msg::ViewTemplate>(
            topic_root + "/LocalView/Template", 10);
        
        // Subscribe to SDR
        sub_sdr_ = this->create_subscription<topological_msgs::msg::SdrStamped>(
            topic_root + "/sdr", 10,
            [this](const topological_msgs::msg::SdrStamped::SharedPtr msg) {
                this->sdr_callback(msg);
            });
        
        RCLCPP_INFO(this->get_logger(), "SpatialViewCellsNode initialized successfully!");
    }

private:
    void sdr_callback(const topological_msgs::msg::SdrStamped::SharedPtr msg) {
        auto callback_start = std::chrono::high_resolution_clock::now();
        
        const std::vector<uint32_t>& activeCellsSparse = msg->sdr;
        
        // Create Roaring Bitmap from active cell indices
        Roaring sdr_sparse_bitmap;
        for (const auto& cell_idx : activeCellsSparse) {
            sdr_sparse_bitmap.add(cell_idx);
        }
        
        RCLCPP_DEBUG(this->get_logger(), 
                     "Created SDR bitmap with %lu active cells", 
                     sdr_sparse_bitmap.cardinality());
        
        // Call on_image_map with the Roaring bitmap
        auto [interval_map, template_id] = lv_->on_image_map(sdr_sparse_bitmap, counter_);
        
        RCLCPP_DEBUG(this->get_logger(), "template_id: %d", template_id);
        
        // Create and publish ViewTemplate message
        auto vt_msg = std::make_shared<topological_msgs::msg::ViewTemplate>();
        vt_msg->header.stamp = msg->header.stamp; // Use the same timestamp as the image message to publish futher in the experience map
        vt_msg->current_id = template_id;
        // vt_msg->relative_rad = lv_->get_relative_rad();
        
        pub_vt_->publish(*vt_msg);
        
        counter_++;
        
        auto callback_end = std::chrono::high_resolution_clock::now();
        auto callback_duration = std::chrono::duration_cast<std::chrono::microseconds>(callback_end - callback_start);
        
        RCLCPP_DEBUG(this->get_logger(), 
                     "  [SpatialViewCells] Total callback time: %.2f ms", 
                     callback_duration.count() / 1000.0);
    }
    
    // LocalViewMatchWithIntervals instance
    std::unique_ptr<LocalViewMatchWithIntervals> lv_;
    
    int counter_;
    
    // ROS2 publishers and subscribers
    rclcpp::Publisher<topological_msgs::msg::ViewTemplate>::SharedPtr pub_vt_;
    rclcpp::Subscription<topological_msgs::msg::SdrStamped>::SharedPtr sub_sdr_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<SpatialViewCellsNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
