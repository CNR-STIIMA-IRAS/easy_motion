cmake_minimum_required(VERSION 3.8)
project(easy_motion_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(easy_motion_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/motion_client.cpp
  src/easy_motion_utils.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  easy_motion_msgs
  geometry_msgs
  trajectory_msgs
  moveit_msgs
  control_msgs
)

add_executable(simple_motion_client examples/simple_motion_client.cpp)
target_link_libraries(simple_motion_client ${PROJECT_NAME})
ament_target_dependencies(simple_motion_client
  rclcpp
  rclcpp_action
  easy_motion_msgs
  geometry_msgs
  trajectory_msgs
  moveit_msgs
  control_msgs
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS simple_motion_client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  easy_motion_msgs
  geometry_msgs
  trajectory_msgs
  moveit_msgs
  control_msgs
)

ament_package()
