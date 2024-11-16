from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Static transform publisher from 'map' to 'camera_depth_optical_frame'
    static_transform_publisher_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_cam1',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_depth_optical_frame'],
        output='log'
    )

    # Static transform publisher from 'map' to 'depth_camera_link'
    static_transform_publisher_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_cam2',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'depth_camera_link'],
        output='log'
    )

    # Your pointcloud_fusion_node
    pointcloud_fusion_node = Node(
        package='pointcloud_fusion',
        executable='pointcloud_fusion_node',
        name='pointcloud_fusion_node',
        output='screen',
        arguments=['/depth/image_raw','/depth/camera_info','/camera/camera/depth/image_rect_raw','/camera/camera/color/camera_info'],
    )

    return LaunchDescription([
        static_transform_publisher_1,
        static_transform_publisher_2,
        pointcloud_fusion_node,
    ])
