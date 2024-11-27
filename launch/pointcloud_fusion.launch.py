from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
     # Static transform publisher for realssense result from calibration
    static_transform_publisher_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_cam1',
        arguments=['-0.0171', '-0.0858', '0.0606', '-0.0287', '-0.0315', '-2.8281', 'link_6', 'camera_link'],
        output='log'
    )
     # Static transform publisher for kinet result from calibration
    static_transform_publisher_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_cam2',
        arguments=['-0.1962', '-0.2486', '0.8843', '-1.4077', '-0.1897', '-0.7588', 'base_link', 'camera_base'],
        output='log'
    )

    # pointcloud_fusion_node
    pointcloud_fusion_node = Node(
        package='pointcloud_fusion',
        executable='pointcloud_fusion_node',
        name='pointcloud_fusion_node',
        output='screen',
        arguments=['/depth/image_raw','/depth/camera_info','/camera/camera/depth/image_rect_raw','/camera/camera/depth/camera_info'],
    )

    return LaunchDescription([
        static_transform_publisher_1,
        static_transform_publisher_2,
        pointcloud_fusion_node,
    ])
