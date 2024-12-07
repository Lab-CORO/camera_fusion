from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    #Tf selon les calibration du Tracker Marker
    tf_computation_node_RS = Node(
            package='camera_calibration',  
            executable='rs_tf_computation_node',  
            name='rs_tf_computation_node',  
            output='screen',  
    ),

    tf_computation_node_Kinect = Node(
            package='camera_calibration',  
            executable='kinect_tf_computation_node',  
            name='kinect_tf_computation_node',  
            output='screen',  
    ),

    # pointcloud_fusion_node 
    pointcloud_fusion_node = Node(
        package='pointcloud_fusion',
        executable='pointcloud_fusion_node',
        name='pointcloud_fusion_node',
        output='screen',
        arguments=[
            '/depth/image_raw',
            '/depth/camera_info',
            '/camera/camera/depth/image_rect_raw',
            '/camera/camera/depth/camera_info'
        ],
    )

    return LaunchDescription([
        tf_computation_node_RS,
        tf_computation_node_Kinect,
        pointcloud_fusion_node,
    ])
