cmake_minimum_required(VERSION 3.8)
project(pointcloud_fusion)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)

set(dependencies_pkgs
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  PCL
  OpenCV
  cv_bridge
  tf2
  tf2_ros
  tf2_eigen  
  geometry_msgs
  message_filters
  image_transport
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)




# Define executable
add_executable(${PROJECT_NAME}_node
  src/pointcloud_fusion.cpp
)

# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node ${dependencies_pkgs})

# Specify C++ standard
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)

# Install
install( TARGETS 
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
  
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_include_directories(include)
# ament_export_targets(${PROJECT_NAME}_node HAS_LIBRARY_TARGET)
# ament_export_dependencies(${dependencies_pkgs})

ament_package()
