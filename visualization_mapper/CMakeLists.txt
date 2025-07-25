cmake_minimum_required(VERSION 3.8)
project(visualization_mapper)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find SDL2
find_package(SDL2 REQUIRED)

# Add executable
add_executable(visualization_mapper
  src/main.cpp
  src/mapper_node.cpp
)

# Include headers
target_include_directories(visualization_mapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SDL2_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(visualization_mapper ${SDL2_LIBRARIES})

# ROS2 dependencies
ament_target_dependencies(visualization_mapper
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# Install target
install(TARGETS visualization_mapper
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
