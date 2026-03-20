#include <rclcpp/rclcpp.hpp>
#include <topological_msgs/msg/binary_features.hpp>
#include <topological_msgs/msg/sdr_stamped.hpp>
#include <memory>
#include <iostream>
#include <chrono>
#include <vector>
#include <array>

#include "htm/algorithms/TemporalMemory.hpp"
#include "htm/types/Sdr.hpp"

/**
 * @brief NeocortexNode - Hierarchical Temporal Memory for temporal sequence learning
 * 
 * This ROS2 node implements HTM's Temporal Memory algorithm to learn temporal sequences
 * in visual feature patterns. It processes binary features and outputs Sparse Distributed
 * Representations (SDR) of winner cells, encoding both current sensory input and temporal
 * context. This enables the system to learn and predict sequences of visual experiences.
 * 
 * Subscriptions:
 *   - <topic_root>/bin_features: Receives binary feature vectors from visual processing
 * 
 * Publications:
 *   - <topic_root>/sdr: Publishes SDR representations with temporal context
 * 
 * Parameters:
 *   - topic_root: Base topic namespace for all subscriptions and publications
 *   - tm_column_dimensions: Number of columns in the Temporal Memory (default: 2048)
 *   - tm_cells_per_column: Number of cells per column (default: 32)
 *   - tm_activation_threshold: Min active synapses to activate a segment (default: 4)
 *   - tm_initial_permanence: Initial permanence value for new synapses (default: 0.55)
 *   - tm_connected_permanence: Permanence threshold for connected synapses (default: 0.5)
 *   - tm_min_threshold: Min active synapses to grow new synapses (default: 1)
 *   - tm_max_new_synapse_count: Max new synapses per segment per step (default: 20)
 *   - tm_permanence_increment: Permanence increase for active synapses (default: 0.01)
 *   - tm_permanence_decrement: Permanence decrease for inactive synapses (default: 0.01)
 *   - tm_predicted_segment_decrement: Permanence decrease for segments in predicted inactive columns (default: 0.0005)
 *   - tm_max_segments_per_cell: Maximum number of segments per cell (default: 100)
 *   - tm_max_synapses_per_segment: Maximum number of synapses per segment (default: 100)
 *   - tm_seed: Random seed for reproducibility (default: 42)
 */
class NeocortexNode : public rclcpp::Node {
public:
    explicit NeocortexNode()
        : Node("neocortex_node") {
        
        // Declare and get parameters
        this->declare_parameter("topic_root", "");
        this->declare_parameter("tm_column_dimensions", 2048);
        this->declare_parameter("tm_cells_per_column", 32);
        this->declare_parameter("tm_activation_threshold", 4);
        this->declare_parameter("tm_initial_permanence", 0.55);
        this->declare_parameter("tm_connected_permanence", 0.5);
        this->declare_parameter("tm_min_threshold", 1);
        this->declare_parameter("tm_max_new_synapse_count", 20);
        this->declare_parameter("tm_permanence_increment", 0.01);
        this->declare_parameter("tm_permanence_decrement", 0.01);
        this->declare_parameter("tm_predicted_segment_decrement", 0.0005);
        this->declare_parameter("tm_max_segments_per_cell", 100);
        this->declare_parameter("tm_max_synapses_per_segment", 100);
        this->declare_parameter("tm_seed", 42);
        
        std::string topic_root = this->get_parameter("topic_root").as_string();
        tm_column_dimensions_ = this->get_parameter("tm_column_dimensions").as_int();
        tm_cells_per_column_ = this->get_parameter("tm_cells_per_column").as_int();
        tm_activation_threshold_ = this->get_parameter("tm_activation_threshold").as_int();
        tm_initial_permanence_ = this->get_parameter("tm_initial_permanence").as_double();
        tm_connected_permanence_ = this->get_parameter("tm_connected_permanence").as_double();
        tm_min_threshold_ = this->get_parameter("tm_min_threshold").as_int();
        tm_max_new_synapse_count_ = this->get_parameter("tm_max_new_synapse_count").as_int();
        tm_permanence_increment_ = this->get_parameter("tm_permanence_increment").as_double();
        tm_permanence_decrement_ = this->get_parameter("tm_permanence_decrement").as_double();
        tm_predicted_segment_decrement_ = this->get_parameter("tm_predicted_segment_decrement").as_double();
        tm_max_segments_per_cell_ = this->get_parameter("tm_max_segments_per_cell").as_int();
        tm_max_synapses_per_segment_ = this->get_parameter("tm_max_synapses_per_segment").as_int();
        tm_seed_ = this->get_parameter("tm_seed").as_int();
        
        RCLCPP_INFO(this->get_logger(), "NeocortexNode Parameters:");

        // Initialize TemporalMemory
        tm_ = std::make_unique<htm::TemporalMemory>(
            std::vector<htm::UInt>{static_cast<htm::UInt>(tm_column_dimensions_)}, // columnDimensions
            tm_cells_per_column_,    // cellsPerColumn
            tm_activation_threshold_,    // activationThreshold
            tm_initial_permanence_,  // initialPermanence
            tm_connected_permanence_,  // connectedPermanence
            tm_min_threshold_,    // minThreshold
            tm_max_new_synapse_count_,   // maxNewSynapseCount
            tm_permanence_increment_,  // permanenceIncrement
            tm_permanence_decrement_,  // permanenceDecrement
            tm_predicted_segment_decrement_,  // predictedSegmentDecrement
            tm_seed_,    // seed
            tm_max_segments_per_cell_,
            tm_max_synapses_per_segment_
        );
        
        RCLCPP_INFO(this->get_logger(), "TemporalMemory initialized with %d columns and %d cells per column", 
                    tm_column_dimensions_, tm_cells_per_column_);
        tm_->printParameters();
        
        // Create publisher for SDR
        pub_sdr_ = this->create_publisher<topological_msgs::msg::SdrStamped>(
            topic_root + "/sdr", 10);
        
        // Subscribe to binary features
        sub_bin_features_ = this->create_subscription<topological_msgs::msg::BinaryFeatures>(
            topic_root + "/bin_features", 10,
            [this](const topological_msgs::msg::BinaryFeatures::SharedPtr msg) {
                this->bin_features_callback(msg);
            });
        
        RCLCPP_INFO(this->get_logger(), "NeocortexNode initialized successfully!");
    }

private:
    void bin_features_callback(const topological_msgs::msg::BinaryFeatures::SharedPtr msg) {
        auto callback_start = std::chrono::high_resolution_clock::now();
        
        // Convert binary features to active column indices
        std::vector<htm::UInt> activeColumnIndices;
        for (size_t i = 0; i < msg->bin_features.size() && i < 2048; ++i) {
            if (msg->bin_features[i] != 0) {
                activeColumnIndices.push_back(static_cast<htm::UInt>(i));
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Active columns count: %zu from total of %zu", 
            activeColumnIndices.size(), msg->bin_features.size());
        
        // Create SDR for active columns
        std::vector<htm::UInt> columnDims = tm_->getColumnDimensions(); // {2048}
        htm::SDR activeColumns(columnDims);
        activeColumns.setSparse(activeColumnIndices);
        
        // Compute temporal memory
        auto tm_start = std::chrono::high_resolution_clock::now();
        tm_->compute(activeColumns, true); // learn=true
        auto tm_end = std::chrono::high_resolution_clock::now();
        auto tm_duration = std::chrono::duration_cast<std::chrono::microseconds>(tm_end - tm_start);
        
        // Get winner cells
        size_t num_columns = tm_->getColumnDimensions()[0];
        size_t num_cells_per_column = tm_->getCellsPerColumn();
        size_t total_cells = num_columns * num_cells_per_column; // 65536
        
        htm::SDR winnerCells({static_cast<htm::UInt>(total_cells)});
        tm_->getWinnerCells(winnerCells);
        const std::vector<htm::UInt>& activeCellsSparse = winnerCells.getSparse();
        
        RCLCPP_INFO(this->get_logger(), 
                    "Winner cells count: %zu (out of %zu total)", 
                    activeCellsSparse.size(), total_cells);
        
        // Publish SDR
        auto sdr_msg = topological_msgs::msg::SdrStamped();
        sdr_msg.header = msg->header; // Preserve original header for synchronization
        sdr_msg.sdr = activeCellsSparse;
        pub_sdr_->publish(sdr_msg);
        
        auto callback_end = std::chrono::high_resolution_clock::now();
        auto callback_duration = std::chrono::duration_cast<std::chrono::microseconds>(callback_end - callback_start);
        
        RCLCPP_DEBUG(this->get_logger(), 
                     "  [NeocortexNode] TM computation: %.2f ms", 
                     tm_duration.count() / 1000.0);
        RCLCPP_DEBUG(this->get_logger(), 
                     "  [NeocortexNode] Total callback time: %.2f ms", 
                     callback_duration.count() / 1000.0);
    }
    
    // TemporalMemory parameters
    int tm_column_dimensions_;
    int tm_cells_per_column_;
    int tm_activation_threshold_;
    double tm_initial_permanence_;
    double tm_connected_permanence_;
    int tm_min_threshold_;
    int tm_max_new_synapse_count_;
    double tm_permanence_increment_;
    double tm_permanence_decrement_;
    double tm_predicted_segment_decrement_;
    int tm_max_segments_per_cell_;
    int tm_max_synapses_per_segment_;
    int tm_seed_;
    
    // HTM TemporalMemory instance
    std::unique_ptr<htm::TemporalMemory> tm_;
    
    // ROS2 publishers and subscribers
    rclcpp::Publisher<topological_msgs::msg::SdrStamped>::SharedPtr pub_sdr_;
    rclcpp::Subscription<topological_msgs::msg::BinaryFeatures>::SharedPtr sub_bin_features_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<NeocortexNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
