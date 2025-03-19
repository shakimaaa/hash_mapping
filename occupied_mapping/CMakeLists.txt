cmake_minimum_required(VERSION 3.8)
project(occupied_mapping)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(debug_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(fmt REQUIRED)
find_package(TBB REQUIRED)

find_package(rm_interface REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
  src/grid_map.cpp
)

target_link_libraries(${PROJECT_NAME} 
  ${PCL_LIBRARIES}
  fmt::fmt
  TBB::tbb
)

ament_target_dependencies(${PROJECT_NAME}
  "debug_tools"
)

add_executable(${PROJECT_NAME}_node
  src/main_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node 
  ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node
  "rclcpp"
  "rclpy"
  "geometry_msgs"
  "nav_msgs"
  "pcl_ros"
  "sensor_msgs"
  "visualization_msgs"
  "cv_bridge"
  "rm_interface"
  "debug_tools"
)

add_executable(pcd_saver_node
  src/saver_pcd.cpp
)

ament_target_dependencies(pcd_saver_node
  "rclcpp"
  "rm_interface"
  "debug_tools"
)

add_executable(${PROJECT_NAME}_node_test
  test/main_test.cpp
)

target_link_libraries(${PROJECT_NAME}_node_test 
  ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node_test
  "rclcpp"
  "rclpy"
  "geometry_msgs"
  "nav_msgs"
  "pcl_ros"
  "sensor_msgs"
  "visualization_msgs"
  "cv_bridge"
)

# install lib file
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install executable file
install(TARGETS 
  ${PROJECT_NAME}_node_test 
  ${PROJECT_NAME}_node
  pcd_saver_node
  DESTINATION lib/${PROJECT_NAME} 
)

# Install config and launch files.
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()

