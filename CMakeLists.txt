cmake_minimum_required(VERSION 3.5)
project(pc2_multi_merger)

# Default to C99
set(CMAKE_C_STANDARD 99)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PCL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  pcl_ros
  PCL
  tf2_geometry_msgs
)

# Build
add_executable(pc2_multi_merger src/pc2_multi_merger_node.cpp)
ament_target_dependencies(pc2_multi_merger ${dependencies})
target_include_directories(pc2_multi_merger PUBLIC ${EIGEN_INCLUDE_DIRS})
target_include_directories(pc2_multi_merger PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(pc2_multi_merger ${PCL_LIBRARIES})


# Install
install(TARGETS pc2_multi_merger
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(${dependencies})
ament_package()
