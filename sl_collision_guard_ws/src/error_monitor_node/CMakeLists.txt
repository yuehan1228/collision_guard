cmake_minimum_required(VERSION 3.16)
project(error_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_executable(error_monitor_node ${SOURCES})

ament_target_dependencies(error_monitor_node
  rclcpp
  pcl_ros
  sensor_msgs
  custom_msgs
)

target_link_libraries(error_monitor_node 
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS
  error_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
