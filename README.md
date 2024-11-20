# Camera Fusion Package

This repository contains a C++ package for fusing point clouds from two different cameras using ROS 2. The package also supports 2D images. It is designed to work with RealSense 2 and Azure Kinect cameras.

# Prerequisites

Before getting started, make sure you have the following:

ROS 2 (compatible version with your dependencies)
Drivers for RealSense 2 and Azure Kinect cameras
Colcon for building ROS 2 packages

# Installation and Usage Steps

1. Clone the repository:
    git clone https://github.com/Lab-CORO/pointcloud_fusion.git
2. Build the package:
    colcon build --packages-select pointcloud_fusion
3. Source the workspace:
    source install/setup.bash
4. Launch the cameras:
. For the RealSense 2 camera:
    ros2 launch realsense2_camera rs_launch.py
. For the Azure Kinect camera:
    ros2 launch driver.launch.py
5. Launch the fusion package:
    ros2 launch pointcloud_fusion pointcloud_fusion.launch.py

# Features

Real-time fusion of point clouds from two cameras.
Support for RealSense 2 and Azure Kinect cameras.
Compatible with 2D images and 3D point clouds using ROS 2.

# Contributors

