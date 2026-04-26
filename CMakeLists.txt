cmake_minimum_required(VERSION 3.8)
project(neoslam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(topological_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(Irrlicht REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(roaring REQUIRED)

# Find Python and pybind11 for visual feature extractor
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# CRITICAL: Enable optimization flags for Eigen performance
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2 -mfma")  # Enable AVX2 and FMA for SIMD
# Eigen optimization flags
add_definitions(-DEIGEN_NO_DEBUG)           # Disable Eigen debug checks
add_definitions(-DEIGEN_DONT_PARALLELIZE)   # Single-threaded (faster for small ops)

# include_directories(
#   ${CMAKE_CURRENT_SOURCE_DIR}/src
#   ${CMAKE_CURRENT_SOURCE_DIR}/src/neocortex
#   ${CMAKE_CURRENT_SOURCE_DIR}/src/spatial_view_cells
#   ${Python3_INCLUDE_DIRS}
#   ${pybind11_INCLUDE_DIRS}
# )

# neoslam library
# add_library(neoslam 
#   src/experience_map/experience_map.cpp 
#   src/pose_cells/posecell_network.cpp 
#   src/visual_odometry/visual_odometry.cpp
# )

# target_include_directories(neoslam PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
#   $<INSTALL_INTERFACE:include>
# )

# ament_target_dependencies(neoslam
#   rclcpp
#   std_msgs
#   sensor_msgs
#   nav_msgs
#   geometry_msgs
#   visualization_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   image_transport
#   cv_bridge
#   topological_msgs
# )

# target_link_libraries(neoslam
#   ${OpenCV_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# uncomment if you don't have irrlicht installed
add_definitions("-DHAVE_IRRLICHT")

# visual_feature_extractor_node executable
add_executable(visual_feature_extractor_node 
  src/visual_feature_extractor/visual_feature_extractor_node.cpp)
target_include_directories(visual_feature_extractor_node PRIVATE
  ${Python3_INCLUDE_DIRS}
  ${pybind11_INCLUDE_DIRS}
)
target_link_libraries(visual_feature_extractor_node 
  pybind11::embed
  Python3::Python
)

ament_target_dependencies(visual_feature_extractor_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  topological_msgs
)

# binary_projector_node executable
add_executable(binary_projector_node 
  src/dim_reduction_and_binarization/binary_projector_node.cpp)
target_include_directories(binary_projector_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(binary_projector_node
  rclcpp
  std_msgs
  topological_msgs
)

# neocortex_node executable
add_executable(neocortex_node src/neocortex/neocortex_node.cpp)
target_include_directories(neocortex_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/neocortex
)
target_link_libraries(neocortex_node 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/neocortex/htm/libhtm_core.a
  pthread
)
ament_target_dependencies(neocortex_node
  rclcpp
  std_msgs
  topological_msgs
)

# spatial_view_cells_node executable
add_executable(spatial_view_cells_node 
  src/spatial_view_cells/spatial_view_cells_node.cpp)
target_include_directories(spatial_view_cells_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/spatial_view_cells
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(spatial_view_cells_node 
  roaring::roaring
)
ament_target_dependencies(spatial_view_cells_node
  rclcpp
  std_msgs
  topological_msgs
)

# pose_cells_node executable
add_executable(pose_cells_node  
  src/pose_cells/pose_cells_node.cpp
  src/pose_cells/posecell_network.cpp)
target_include_directories(pose_cells_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pose_cells
)
target_link_libraries(pose_cells_node
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES}
)
ament_target_dependencies(pose_cells_node
  rclcpp
  nav_msgs
  topological_msgs
)

# experience_map_node executable
add_executable(experience_map_node 
  src/experience_map/experience_map_node.cpp
  src/experience_map/experience_map.cpp)
target_include_directories(experience_map_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/experience_map
)
target_link_libraries(experience_map_node
  ${Boost_LIBRARIES}
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES}
)
ament_target_dependencies(experience_map_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  topological_msgs
)

# visual_odometry_node executable
add_executable(visual_odometry_node 
  src/visual_odometry/visual_odometry_node.cpp
  src/visual_odometry/visual_odometry.cpp)
  target_include_directories(visual_odometry_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/visual_odometry
)
target_link_libraries(visual_odometry_node
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(visual_odometry_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_transport
  cv_bridge
  topological_msgs
)

# steering_model_node executable
add_executable(steering_model_node 
  src/steering_model/steering_model_node.cpp
  src/steering_model/rosi.cpp)
target_include_directories(steering_model_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/steering_model
)
target_link_libraries(steering_model_node
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES}
)
ament_target_dependencies(steering_model_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# odom_fusion_node executable
add_executable(odom_fusion_node 
  src/odom_fusion/odom_fusion_node.cpp)
target_include_directories(odom_fusion_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/odom_fusion
)
ament_target_dependencies(odom_fusion_node
  rclcpp
  std_msgs
  nav_msgs
)


# Install targets
# install(TARGETS neoslam
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(TARGETS 
  visual_feature_extractor_node
  binary_projector_node
  neocortex_node
  spatial_view_cells_node 
  pose_cells_node 
  experience_map_node 
  visual_odometry_node
  steering_model_node
  odom_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY src/media/
  DESTINATION share/${PROJECT_NAME}/media
)

# Install Python module for visual feature extractor
install(DIRECTORY src/visual_feature_extractor/
  DESTINATION lib/${PROJECT_NAME}/visual_feature_extractor
  FILES_MATCHING PATTERN "*.py"
)

# Install random matrix for binary projector
install(DIRECTORY src/dim_reduction_and_binarization/random_matrix/
  DESTINATION share/${PROJECT_NAME}/random_matrix
  FILES_MATCHING PATTERN "*.bin"
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# Install headers
install(DIRECTORY src/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
