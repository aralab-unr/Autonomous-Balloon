cmake_minimum_required(VERSION 3.8)
project(esp8266server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add the server node
add_executable(server src/server.cpp)
ament_target_dependencies(server std_msgs rclcpp)

# Install the executable
install(TARGETS server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
