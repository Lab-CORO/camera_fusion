from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Transformation de 'link_6' vers 'camera_link' (caméra RealSense)
    static_transform_publisher_link6_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_link6_to_camera_link',
        arguments=[
            #represent neccesary transform acquired with the calibration
            '-0.0171',  # x
            '-0.0858',  # y
            '0.0606',   # z
            '-0.0287',  # roll  (en radians)
            '-0.0315',  # pitch (en radians)
            '-2.8281', # yaw   (en radians)
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
            '0',  # x
            '0',  # y
            '0',   # z
            '0',  # roll  (en radians)
            '0',  # pitch (en radians)
            '0',  # yaw   (en radians)
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
