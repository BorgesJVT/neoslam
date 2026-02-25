#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <chrono>

namespace py = pybind11;

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
    
    std::string topic_root = this->get_parameter("topic_root").as_string();
    std::string python_module_path = this->get_parameter("python_module_path").as_string();
    bool use_compressed = this->get_parameter("use_compressed").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "VisualFeatureExtractor Parameters:");
    RCLCPP_INFO(this->get_logger(), "  topic_root: %s", topic_root.c_str());
    RCLCPP_INFO(this->get_logger(), "  python_module_path: %s", python_module_path.c_str());
    RCLCPP_INFO(this->get_logger(), "  use_compressed: %s", use_compressed ? "true" : "false");
    
    // Initialize Python wrapper
    try {
      initialize_python(python_module_path);
      RCLCPP_INFO(this->get_logger(), "Python module initialized successfully!");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize Python module: %s", e.what());
      throw;
    }
    
    // Create publisher for feature vectors
    pub_features_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
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
  void initialize_python(const std::string& python_module_path)
  {
    // Add Python module path to sys.path
    py::module sys = py::module::import("sys");
    sys.attr("path").attr("insert")(1, python_module_path);
    
    // Import the Python module and create instance
    vision_module_ = py::module::import("visual_feature_extractor_node");
    vision_class_ = vision_module_.attr("VisualFeatureExtractorNode");
    vision_instance_ = vision_class_();
    
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
      publish_features(features_vector);
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
      publish_features(features_vector);
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
  
  void publish_features(const std::vector<float>& features)
  {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = features;
    pub_features_->publish(msg);
  }
  
  // ROS2 publishers and subscribers
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_features_;
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
