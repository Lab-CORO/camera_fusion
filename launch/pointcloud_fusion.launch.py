from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Static transform publisher from 'map' to 'camera_depth_optical_frame'
    static_transform_publisher_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_cam1',
        arguments=['0', '-1', '0', '0', '0', '0', 'camera_link', 'camera_base'],
        output='log'
    )

    # Your pointcloud_fusion_node
    pointcloud_fusion_node = Node(
        package='pointcloud_fusion',
        executable='pointcloud_fusion_node',
        name='pointcloud_fusion_node',
        output='screen',
        arguments=['/depth/image_raw','/depth/camera_info','/camera/camera/depth/image_rect_raw','/camera/camera/depth/camera_info'],
    )

    return LaunchDescription([
        static_transform_publisher_1,
        pointcloud_fusion_node,
    ])
