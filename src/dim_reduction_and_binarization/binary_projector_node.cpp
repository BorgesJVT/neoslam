#include <rclcpp/rclcpp.hpp>
#include <topological_msgs/msg/visual_features.hpp>
#include <topological_msgs/msg/binary_features.hpp>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <array>
#include <vector>
#include <algorithm>
#include <cmath>

class BinaryProjector : public rclcpp::Node {
public:
    explicit BinaryProjector()
        : Node("binary_projector_node") {
        
        // Declare and get parameters
        this->declare_parameter("topic_root", "");
        this->declare_parameter("random_matrix_path", "");
        
        std::string topic_root = this->get_parameter("topic_root").as_string();
        std::string random_matrix_path = this->get_parameter("random_matrix_path").as_string();
        
        RCLCPP_INFO(this->get_logger(), "BinaryProjector Parameters:");
        RCLCPP_INFO(this->get_logger(), "  topic_root: %s", topic_root.c_str());
        RCLCPP_INFO(this->get_logger(), "  random_matrix_path: %s", random_matrix_path.c_str());
        
        // Load random matrix for LSBH computation
        load_random_matrix(random_matrix_path);
        
        // Create publisher for binary features
        pub_bin_features_ = this->create_publisher<topological_msgs::msg::BinaryFeatures>(
            topic_root + "/bin_features", 10);
        
        // Subscribe to visual features
        sub_visual_features_ = this->create_subscription<topological_msgs::msg::VisualFeatures>(
            topic_root + "/visual_features", 10,
            [this](const topological_msgs::msg::VisualFeatures::SharedPtr msg) {
                this->visual_features_callback(msg);
            });
        
        RCLCPP_INFO(this->get_logger(), "BinaryProjector initialized successfully!");
    }

private:
    void visual_features_callback(const topological_msgs::msg::VisualFeatures::SharedPtr msg) {
        auto callback_start = std::chrono::high_resolution_clock::now();

        // Get feature vector from message
        const std::vector<float>& features_vector = msg->features;
        
        if (features_vector.size() != FEATURE_DIM) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Feature vector size mismatch! Expected %d, got %zu", 
                        FEATURE_DIM, features_vector.size());
            return;
        }
        
        // Compute LSBH from features
        std::array<bool, 2048> bin_features = get_lsbh(features_vector, 0.25);

        // Publish binary features
        auto bin_features_msg = topological_msgs::msg::BinaryFeatures();
        bin_features_msg.bin_features.reserve(2048); 
        for (const auto& bit : bin_features) {
            bin_features_msg.bin_features.push_back(bit ? 1 : 0); 
        }

        bin_features_msg.header = msg->header; // Preserve original header for synchronization
        
        // Count and log active bits
        int active_bits = std::count(bin_features.begin(), bin_features.end(), true);
        RCLCPP_DEBUG(this->get_logger(), 
                    "[BinaryProjector] Binary features: %d active bits out of %d", 
                    active_bits, static_cast<int>(bin_features.size()));
        
        pub_bin_features_->publish(bin_features_msg);
        
        auto callback_end = std::chrono::high_resolution_clock::now();
        auto callback_duration = std::chrono::duration_cast<std::chrono::microseconds>(callback_end - callback_start);
        
        
        RCLCPP_INFO(this->get_logger(), 
          "counter: %u. Total callback time: %.2f ms", counter_, callback_duration.count() / 1000.0);
        
        counter_++;
    }
    
    /**
     * @brief Compute Locality-Sensitive Binary Hashing (LSBH) from feature vector
     * 
     * LSBH is a dimensionality reduction technique that converts high-dimensional features
     * into binary codes while preserving locality (similar features get similar codes).
     * 
     * Algorithm:
     * 1. Random Projection: Project features onto random hyperplanes (matrix multiplication)
     * 2. Selection: Pick top 25% and bottom 25% of projected values
     * 3. Binary Encoding: Create binary hash where selected indices are set to 1
     * 
     * @param features High-dimensional feature vector (64896 elements from AlexNet Conv3)
     * @param threshold_percentage Percentage of dimensions to select (0.25 = top/bottom 25%)
     * @return Binary hash array of 2048 bits (1024 top + 1024 bottom)
     */
    std::array<bool, 2048> get_lsbh(const std::vector<float>& features, float threshold_percentage) {
        // Calculate how many dimensions to select (25% of 1024 = 256 dimensions)
        int num_selected_dimensions = static_cast<int>(std::round(PROJECTION_DIM * threshold_percentage));
        
        // Step 1: Create Eigen view of feature vector (zero-copy)
        Eigen::Map<const Eigen::RowVectorXf> feature_row_vector(features.data(), features.size());
        
        // Verify dimensions match for matrix multiplication
        if (feature_row_vector.size() != FEATURE_DIM) {
            RCLCPP_ERROR(this->get_logger(), 
                         "Feature dimension mismatch! Got %ld but expected %d", 
                         feature_row_vector.size(), FEATURE_DIM);
        }
        
        // Step 2: Random Projection (dimensionality reduction)
        // Project high-dimensional features (64896) to lower dimensions (1024)
        // Operation: (1 x 64896) * (64896 x 1024) = (1 x 1024)
        Eigen::RowVectorXf projected_features = feature_row_vector * random_matrix_;
        
        // Step 3: Create (value, index) pairs for sorting
        std::vector<std::pair<float, int>> value_index_pairs;
        value_index_pairs.reserve(projected_features.size());
        for (int dimension_idx = 0; dimension_idx < projected_features.size(); ++dimension_idx) {
            value_index_pairs.emplace_back(projected_features(dimension_idx), dimension_idx);
        }
        
        // Step 4: Partial sort to find top and bottom dimensions
        // Sort to find bottom 25% (smallest values)
        auto bottom_threshold = value_index_pairs.begin() + num_selected_dimensions;
        std::partial_sort(value_index_pairs.begin(), bottom_threshold, value_index_pairs.end());
        
        // Sort to find top 25% (largest values)
        auto top_threshold = value_index_pairs.rbegin() + num_selected_dimensions;
        std::partial_sort(value_index_pairs.rbegin(), top_threshold, value_index_pairs.rend());
        
        // Step 5: Create binary hash array (2048 bits = 1024 top + 1024 bottom)
        std::array<bool, 2048> binary_hash;
        binary_hash.fill(false);  // Initialize all bits to 0
        
        // Set bits for top 25% dimensions (indices 0-1023)
        for (int i = 0; i < num_selected_dimensions; ++i) {
            int dimension_idx = value_index_pairs[value_index_pairs.size() - 1 - i].second;
            binary_hash[dimension_idx] = true;
        }
        
        // Set bits for bottom 25% dimensions (indices 1024-2047)
        for (int i = 0; i < num_selected_dimensions; ++i) {
            int dimension_idx = value_index_pairs[i].second;
            binary_hash[dimension_idx + PROJECTION_DIM] = true;
        }
        
        return binary_hash;
    }
    
    /**
     * @brief Load random projection matrix from binary file
     * 
     * Loads a raw binary file containing float32 data in row-major order.
     * Matrix dimensions are fixed: 64896 x 1024
     * 
     * @param bin_path Path to binary file containing random matrix
     */
    void load_random_matrix(const std::string& bin_path) {
        auto time_start = std::chrono::high_resolution_clock::now();
        
        // Open binary file
        std::ifstream bin_file(bin_path, std::ios::binary);
        if (!bin_file.is_open()) {
            throw std::runtime_error("Cannot open random matrix file: " + bin_path);
        }
        
        // Allocate matrix with known dimensions
        random_matrix_.resize(FEATURE_DIM, PROJECTION_DIM);
        
        // Calculate expected file size
        size_t expected_elements = FEATURE_DIM * PROJECTION_DIM;
        size_t expected_bytes = expected_elements * sizeof(float);
        
        // Get actual file size
        bin_file.seekg(0, std::ios::end);
        size_t file_size = bin_file.tellg();
        bin_file.seekg(0, std::ios::beg);
        
        // Verify file size matches expected dimensions
        if (file_size != expected_bytes) {
            throw std::runtime_error(
                "Random matrix file size mismatch! Expected " + 
                std::to_string(expected_bytes) + " bytes (" + 
                std::to_string(FEATURE_DIM) + "x" + std::to_string(PROJECTION_DIM) + 
                "), got " + std::to_string(file_size) + " bytes"
            );
        }
        
        // Read binary data directly into Eigen matrix memory
        bin_file.read(reinterpret_cast<char*>(random_matrix_.data()), expected_bytes);
        
        if (!bin_file) {
            throw std::runtime_error("Failed to read random matrix file completely");
        }
        
        bin_file.close();
        
        auto time_end = std::chrono::high_resolution_clock::now();
        double load_time_ms = std::chrono::duration<double, std::milli>(time_end - time_start).count();
        
        RCLCPP_INFO(this->get_logger(), "Random matrix loaded from binary: %d x %d", 
                    FEATURE_DIM, PROJECTION_DIM);
        RCLCPP_INFO(this->get_logger(), "File: %s", bin_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Load time: %.2f ms", load_time_ms);
        RCLCPP_INFO(this->get_logger(), "Matrix size: %.2f MB", 
                    (expected_bytes / 1024.0 / 1024.0));
    }

    // LSBH parameters - Fixed dimensions for AlexNet Conv3 output
    static constexpr int FEATURE_DIM = 64896;  // AlexNet Conv3: 384 x 13 x 13
    static constexpr int PROJECTION_DIM = 1024; // Target dimensionality after random projection
    
    // Random matrix for LSBH (using Eigen for performance)
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> random_matrix_;
    
    // ROS2 publishers and subscribers
    rclcpp::Publisher<topological_msgs::msg::BinaryFeatures>::SharedPtr pub_bin_features_;
    rclcpp::Subscription<topological_msgs::msg::VisualFeatures>::SharedPtr sub_visual_features_;
    uint16_t counter_ = 0;  // For debugging and logging purposes
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<BinaryProjector>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
