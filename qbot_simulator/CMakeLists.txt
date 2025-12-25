cmake_minimum_required(VERSION 3.8)
project(qbot_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

include_directories(include)

add_executable(sensor_frame_fix src/sensor_frame_fix.cpp)
ament_target_dependencies(
  sensor_frame_fix
  rclcpp
  sensor_msgs
)

install(TARGETS
  sensor_frame_fix
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY worlds launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
