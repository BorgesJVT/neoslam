#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <topological_msgs/msg/visual_features.hpp>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <chrono>

namespace py = pybind11;

/**
 * @brief VisualFeatureExtractorNode - Deep learning feature extraction from camera images
 * 
 * This ROS2 node extracts high-dimensional visual features from camera images using
 * a pre-trained AlexNet CNN (Conv3 layer output). It embeds a Python interpreter via
 * PyBind11 to leverage PyTorch for GPU-accelerated feature extraction. The node supports
 * both raw and compressed images, optional image cropping, and frame stride control
 * for processing efficiency.
 * 
 * Subscriptions:
 *   - <topic_root>/camera/image/compressed: Receives compressed camera images
 *   - <topic_root>/camera/image: Receives raw camera images (alternative)
 * 
 * Publications:
 *   - <topic_root>/visual_features: Publishes 64,896-dimensional feature vectors
 * 
 * Parameters:
 *   - topic_root: Base topic namespace for all subscriptions and publications
 *   - python_module_path: Path to Python module containing AlexNet feature extractor
 *   - use_compressed: Use compressed (JPEG) images instead of raw images
 *   - frame_stride: Process every Nth frame (1 = all frames)
 *   - crop_image: Enable image cropping before feature extraction
 *   - image_crop_x_min: Minimum X coordinate for image crop (pixels)
 *   - image_crop_x_max: Maximum X coordinate for image crop (-1 = image width)
 *   - image_crop_y_min: Minimum Y coordinate for image crop (pixels)
 *   - image_crop_y_max: Maximum Y coordinate for image crop (-1 = image height)
 */
class VisualFeatureExtractorNode : public rclcpp::Node
{
public:
  VisualFeatureExtractorNode()
    : Node("visual_feature_extractor")
  {
    // Declare parameters
    this->declare_parameter("topic_root", "");
    this->declare_parameter("python_module_path", "");
    this->declare_parameter("use_compressed", true);
    this->declare_parameter("frame_stride", 1);
    this->declare_parameter("crop_image", false);
    this->declare_parameter("image_crop_x_min", 0);
    this->declare_parameter("image_crop_x_max", -1);
    this->declare_parameter("image_crop_y_min", 0);
    this->declare_parameter("image_crop_y_max", -1);
    
    std::string topic_root = this->get_parameter("topic_root").as_string();
    std::string python_module_path = this->get_parameter("python_module_path").as_string();
    bool use_compressed = this->get_parameter("use_compressed").as_bool();
    bool crop_image = this->get_parameter("crop_image").as_bool();
    int image_crop_x_min = this->get_parameter("image_crop_x_min").as_int();
    int image_crop_x_max = this->get_parameter("image_crop_x_max").as_int();
    int image_crop_y_min = this->get_parameter("image_crop_y_min").as_int();
    int image_crop_y_max = this->get_parameter("image_crop_y_max").as_int();
    int frame_stride = this->get_parameter("frame_stride").as_int();
    
    RCLCPP_INFO(this->get_logger(), "VisualFeatureExtractor Parameters:");
    RCLCPP_INFO(this->get_logger(), "  topic_root: %s", topic_root.c_str());
    RCLCPP_INFO(this->get_logger(), "  python_module_path: %s", python_module_path.c_str());
    RCLCPP_INFO(this->get_logger(), "  use_compressed: %s", use_compressed ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  frame_stride: %d", frame_stride);
    RCLCPP_INFO(this->get_logger(), "  crop_image: %s", crop_image ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  image_crop_x_min: %d", image_crop_x_min);
    RCLCPP_INFO(this->get_logger(), "  image_crop_x_max: %d", image_crop_x_max);
    RCLCPP_INFO(this->get_logger(), "  image_crop_y_min: %d", image_crop_y_min);
    RCLCPP_INFO(this->get_logger(), "  image_crop_y_max: %d", image_crop_y_max);
    
    // Initialize Python wrapper
    try {
      initialize_python(python_module_path, frame_stride, crop_image, 
                        image_crop_x_min, image_crop_x_max,
                        image_crop_y_min, image_crop_y_max);
      RCLCPP_INFO(this->get_logger(), "Python module initialized successfully!");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize Python module: %s", e.what());
      throw;
    }
    
    // Create publisher for feature vectors
    pub_features_ = this->create_publisher<topological_msgs::msg::VisualFeatures>(
      topic_root + "/visual_features", 10);
    
    // Subscribe to image topics based on parameter
    if (use_compressed) {
      sub_compressed_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        topic_root + "/camera/image/compressed", 10,
        [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) { 
          this->image_callback_compressed(msg); 
        });
      RCLCPP_INFO(this->get_logger(), "Subscribed to compressed images: %s", 
                  (topic_root + "/camera/image/compressed").c_str());
    } else {
      sub_raw_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_root + "/camera/image", 10,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { 
          this->image_callback_raw(msg); 
        });
      RCLCPP_INFO(this->get_logger(), "Subscribed to raw images: %s", 
                  (topic_root + "/camera/image").c_str());
    }
  }
  
  ~VisualFeatureExtractorNode()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down VisualFeatureExtractorNode");
  }

private:
  void initialize_python(const std::string& python_module_path, int frame_stride,
                        bool crop_image, int image_crop_x_min, int image_crop_x_max,
                        int image_crop_y_min, int image_crop_y_max)
  {
    // Add Python module path to sys.path
    py::module sys = py::module::import("sys");
    sys.attr("path").attr("insert")(1, python_module_path);
    
    // Import the Python module and create instance with parameters
    vision_module_ = py::module::import("visual_feature_extractor_node");
    vision_class_ = vision_module_.attr("VisualFeatureExtractorNode");
    vision_instance_ = vision_class_(frame_stride, crop_image, 
                                      image_crop_x_min, image_crop_x_max,
                                      image_crop_y_min, image_crop_y_max);
    
    std::cout << "[VisualFeatureExtractorNode] Python module initialized successfully!" << std::endl;
  }
  
  void image_callback_compressed(const sensor_msgs::msg::CompressedImage::SharedPtr& img_msg)
  {
    auto t_total_start = std::chrono::high_resolution_clock::now();
    
    try {
      // Call Python to get feature vector
      auto t_python_start = std::chrono::high_resolution_clock::now();
      py::bytes img_bytes(reinterpret_cast<const char*>(img_msg->data.data()), img_msg->data.size());
      std::string format = img_msg->format;
      
      py::object features_obj = vision_instance_.attr("image_callback_wrapper_compressed")(img_bytes, format);
      py::array_t<float> features_array = features_obj.cast<py::array_t<float>>();
      auto t_python_end = std::chrono::high_resolution_clock::now();
      
      // Convert to std::vector
      auto t_convert_start = std::chrono::high_resolution_clock::now();
      const float* features_ptr = features_array.data();
      size_t features_size = features_array.size();
      std::vector<float> features_vector(features_ptr, features_ptr + features_size);
      auto t_convert_end = std::chrono::high_resolution_clock::now();
      
      if (features_size != 64896) {
        RCLCPP_WARN(this->get_logger(), 
                    "Feature vector size is not 64896! Got: %zu", features_size);
      }
      
      // Publish feature vector
      auto t_publish_start = std::chrono::high_resolution_clock::now();
      publish_features(img_msg->header, features_vector);
      auto t_publish_end = std::chrono::high_resolution_clock::now();
      
      auto t_total_end = std::chrono::high_resolution_clock::now();
      
      // Print timing information
      double total_time = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();
      
      RCLCPP_INFO(this->get_logger(), 
        "Counter: %d. Total wrapper time: %.2f ms", counter_, total_time);
      counter_++;
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing compressed image: %s", e.what());
    }
  }
  
  void image_callback_raw(const sensor_msgs::msg::Image::SharedPtr& img_msg)
  {
    auto t_total_start = std::chrono::high_resolution_clock::now();
    
    try {
      // Call Python to get feature vector
      auto t_python_start = std::chrono::high_resolution_clock::now();
      py::bytes img_bytes(reinterpret_cast<const char*>(img_msg->data.data()), img_msg->data.size());
      std::string encoding = img_msg->encoding;
      int width = img_msg->width;
      int height = img_msg->height;
      
      py::object features_obj = vision_instance_.attr("image_callback_wrapper")(img_bytes, width, height, encoding);
      py::array_t<float> features_array = features_obj.cast<py::array_t<float>>();
      auto t_python_end = std::chrono::high_resolution_clock::now();
      
      // Convert to std::vector
      auto t_convert_start = std::chrono::high_resolution_clock::now();
      const float* features_ptr = features_array.data();
      size_t features_size = features_array.size();
      std::vector<float> features_vector(features_ptr, features_ptr + features_size);
      auto t_convert_end = std::chrono::high_resolution_clock::now();
      
      if (features_size != 64896) {
        RCLCPP_WARN(this->get_logger(), 
                    "Feature vector size is not 64896! Got: %zu", features_size);
      }
      
      // Publish feature vector
      auto t_publish_start = std::chrono::high_resolution_clock::now();
      publish_features(img_msg->header, features_vector);
      auto t_publish_end = std::chrono::high_resolution_clock::now();
      
      auto t_total_end = std::chrono::high_resolution_clock::now();
      
      // Print timing information
      double total_time = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();
      
      RCLCPP_INFO(this->get_logger(), 
        "Counter: %d. Total wrapper time: %.2f ms", counter_, total_time);
      counter_++;
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing raw image: %s", e.what());
    }
  }
  
  void publish_features(const std_msgs::msg::Header& header, const std::vector<float>& features)
  {
    auto msg = topological_msgs::msg::VisualFeatures();
    msg.header = header;
    msg.features = features;
    pub_features_->publish(msg);
  }
  
  // ROS2 publishers and subscribers
  rclcpp::Publisher<topological_msgs::msg::VisualFeatures>::SharedPtr pub_features_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_raw_;
  uint32_t counter_ = 0;  // For logging purposes
  
  // Python objects
  py::object vision_module_;
  py::object vision_class_;
  py::object vision_instance_;
};

int main(int argc, char** argv)
{
  // Initialize Python interpreter before ROS2
  py::scoped_interpreter guard{};
  
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<VisualFeatureExtractorNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
