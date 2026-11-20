cmake_minimum_required(VERSION 3.16)
project(plc_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${LIBMODBUS_INCLUDE_DIRS}
)

add_executable(plc_simulator_node
  src/plc_simulator_node.cpp
)

ament_target_dependencies(plc_simulator_node
  rclcpp
  custom_msgs
)

target_link_libraries(plc_simulator_node
  ${LIBMODBUS_LIBRARIES}
)

install(TARGETS
  plc_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/plc_simulator_hmi.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
