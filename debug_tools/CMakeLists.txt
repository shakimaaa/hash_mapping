cmake_minimum_required(VERSION 3.8)
project(debug_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)

# 包含头文件路径  
include_directories(include)

# 外部调用静态库时，不能调用模板类
# add_library(${PROJECT_NAME} SHARED
#   src/color_print.cpp
# )

# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# ament_target_dependencies(${PROJECT_NAME}
#   "rclcpp"
# )

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

ament_export_include_directories(include)  # Export the include directories   

install(DIRECTORY include/  
  DESTINATION include
)
 
# ament_export_libraries(${PROJECT_NAME})
# install(
#   TARGETS ${PROJECT_NAME}
#   LIBRARY DESTINATION lib
# )

ament_package()
