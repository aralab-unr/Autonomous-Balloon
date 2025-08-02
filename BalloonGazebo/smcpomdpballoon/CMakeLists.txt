# Minimum CMake version
cmake_minimum_required(VERSION 3.16)

# Project name and language
project(smcpomdpballoon LANGUAGES C CXX)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)


# Compiler flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# Add the ROS2 node
add_executable(altitudeholding ${SOURCES} src/altitudeholding.cpp)
target_link_libraries(altitudeholding m stdc++)
ament_target_dependencies(altitudeholding rclcpp std_msgs tf2_ros nav_msgs)

add_executable(stationkeeping ${SOURCES} src/stationkeeping.cpp)
target_link_libraries(stationkeeping m stdc++)
ament_target_dependencies(stationkeeping rclcpp std_msgs tf2_ros nav_msgs)

add_library(smcpomdpballoonplugin SHARED src/smcpomdpballoonplugin.cc)
ament_target_dependencies(smcpomdpballoonplugin rclcpp std_msgs)
target_link_libraries(smcpomdpballoonplugin ${GAZEBO_LIBRARIES} rclcpp::rclcpp)

# Install the node
install(TARGETS
  altitudeholding
  stationkeeping
  smcpomdpballoonplugin
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_package()
