cmake_minimum_required(VERSION 3.16)
project(plc_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${LIBMODBUS_INCLUDE_DIRS}
)

add_executable(plc_interface_node
  src/plc_interface_node.cpp
  src/modbus_client.cpp
)

ament_target_dependencies(plc_interface_node
  rclcpp
  sensor_msgs
  custom_msgs
)

target_link_libraries(plc_interface_node 
  ${YAML_CPP_LIBRARIES}
  ${LIBMODBUS_LIBRARIES}
)

install(TARGETS
  plc_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
