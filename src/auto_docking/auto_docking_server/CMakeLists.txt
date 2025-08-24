cmake_minimum_required(VERSION 3.5)
project(auto_docking VERSION 1.1.4 LANGUAGES CXX)

# Compile as C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------------------------------
# Find dependencies
# -------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(robot_msg REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(yaml-cpp REQUIRED)

# -------------------------------
# Include directories
# -------------------------------
include_directories(
  include
)

# -------------------------------
# Add executable
# -------------------------------
add_executable(${PROJECT_NAME}_node
  src/auto_docking_node.cpp
  src/auto_docking_server.cpp
  src/laser_obstacle_detect.cpp
  src/dockingmachine.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  robot_msg
  laser_geometry
  yaml-cpp
)

# -------------------------------
# Install
# -------------------------------
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}/param
)

install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}/cfg
)

# -------------------------------
# Doxygen documentation (optional)
# -------------------------------
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/cfg/Doxyfile.in)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
  add_custom_target(${PROJECT_NAME}_docs ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/docs
    COMMENT "Generating ${PROJECT_NAME} documentation with Doxygen"
    VERBATIM)
else()
  message("Doxygen not found. Skipping documentation generation.")
endif()

# -------------------------------
# Testing (optional)
# -------------------------------
if(BUILD_TEST)
  add_subdirectory(test)
endif()

# -------------------------------
# Export package
# -------------------------------
ament_package()
