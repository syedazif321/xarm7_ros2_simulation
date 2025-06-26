# cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# find_package(gazebo REQUIRED)
# include_directories(${GAZEBO_INCLUDE_DIRS})
# link_directories(${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# add_library(attach_detach SHARED attach_detach.cc)
# target_link_libraries(attach_detach ${GAZEBO_LIBRARIES})


cmake_minimum_required(VERSION 3.5)

project(attach_detach)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Find ROS 2
find_package(rclcpp REQUIRED)
find_package(msg_gazebo REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Add the plugin
add_library(attach_detach SHARED attach_detach.cc)

# Link against ROS 2 and Gazebo libraries
ament_target_dependencies(attach_detach
  rclcpp msg_gazebo
)

target_link_libraries(attach_detach
  ${GAZEBO_LIBRARIES}
)

# Install the plugin
install(
  TARGETS attach_detach
  LIBRARY DESTINATION lib
)

# Enable ament_cmake
ament_package()
