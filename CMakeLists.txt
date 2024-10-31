cmake_minimum_required(VERSION 3.5)
project(pointcloud_fusion)

# Trouver les dépendances du projet
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Ajouter les fichiers source
add_executable(pointcloud_fusion_node src/pointcloud_fusion.cpp)

# Lier les bibliothèques PCL et ROS à l'exécutable
ament_target_dependencies(pointcloud_fusion_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  PCL
)

# Spécifier le standard C++
target_compile_features(pointcloud_fusion_node PUBLIC cxx_std_17)

# Exporter les bibliothèques
install(TARGETS
  pointcloud_fusion_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
