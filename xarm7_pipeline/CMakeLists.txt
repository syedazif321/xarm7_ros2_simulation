cmake_minimum_required(VERSION 3.8)
project(xarm7_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(xarm7_pipeline_node src/xarm7_pipeline_node.cpp)
ament_target_dependencies(xarm7_pipeline_node rclcpp geometry_msgs xarm_msgs yaml-cpp)

install(TARGETS xarm7_pipeline_node DESTINATION lib/${PROJECT_NAME})
# This line links yaml-cpp library properly
target_link_libraries(xarm7_pipeline_node yaml-cpp)
ament_package()
