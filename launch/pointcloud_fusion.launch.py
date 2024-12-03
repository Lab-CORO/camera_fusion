from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Transformation de 'link_6' vers 'camera_link' (caméra RealSense)
    static_transform_publisher_link6_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_link6_to_camera_link',
        arguments=[
            '0',        # x
            '-0.0791',  # y
            '0.03255',  # z
            '1.5708',   # yaw (en radians)
            '-1.2491',  # pitch (en radians)
            '0',        #  roll  (en radians)
            'link_6',
            'camera_link'
        ],
        output='log'
    )

    # Transformation de 'base_link' vers 'camera_base' (caméra Azure Kinect)
    static_transform_publisher_base_link_to_camera_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_link_to_camera_base',
        arguments=[
            '-0.2062',  # x
            '-0.2086',  # y
            '0.8843',   # z
            '0.3666',   # yaw    (en radians)
            '0.7854',   # pitch (en radians)
            '0.2668',   # roll   (en radians)
            'base_link',
            'camera_base'
        ],
        output='log'
    )

    # pointcloud_fusion_node (inchangé)
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
        static_transform_publisher_link6_to_camera_link,
        static_transform_publisher_base_link_to_camera_base,
        pointcloud_fusion_node,
    ])
