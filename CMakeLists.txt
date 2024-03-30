cmake_minimum_required(VERSION 3.8)
project(cmps12)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

#include dirs
include_directories(include)

#add executables
add_executable(orientation_publisher src/orientation_publisher.cpp src/cmps12_i2c.cpp)
add_executable(orientation_subscriber src/orientation_subscriber.cpp src/cmps12_i2c.cpp)

#dependencies
ament_target_dependencies(orientation_publisher rclcpp std_msgs sensor_msgs)
ament_target_dependencies(orientation_subscriber rclcpp std_msgs sensor_msgs)

install(TARGETS
  orientation_publisher
  orientation_subscriber
  DESTINATION lib/cmps12
)

ament_package()
