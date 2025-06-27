cmake_minimum_required(VERSION 3.8)
project(xarm7_pose_saver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(pose_saver src/pose_saver.cpp)

ament_target_dependencies(pose_saver
  rclcpp
  sensor_msgs
  moveit_msgs
  geometry_msgs
  yaml-cpp
  ament_index_cpp
)

target_link_libraries(pose_saver yaml-cpp)

install(TARGETS pose_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# ros2 launch xarm_moveit_config xarm7_moveit_gazebo.launch.py


# run uisng: ros2 run xarm7_pose_saver pose_saver

