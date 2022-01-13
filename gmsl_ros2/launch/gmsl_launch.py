"""Launch a Node with parameters and remappings."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Set parameters
    namespace = LaunchConfiguration('namespace', default='camera')
    node_name = LaunchConfiguration('node_name', default='gmsl_camera_node')
    frame_id = LaunchConfiguration('frame_id', default='gmsl_camera_frame')
    camera_name = LaunchConfiguration('camera_name', default='gmsl_camera')
    camera_config = LaunchConfiguration('camera_config', default='file://' + os.path.join(get_package_share_directory('gmsl_ros2'), 'cfg', 'calibration_param_example.yaml'))
    camera_dev = LaunchConfiguration('camera_dev', default='/dev/video0')

    # Camera node
    camera_node = Node(
        package='gmsl_ros2',
        executable='gmsl_main',
        output='screen',
        name=node_name,
        namespace=namespace,
        parameters=[
                    {
                        'gst_config': (['v4l2src device=', camera_dev, ' ! videoconvert']),
                        'preroll': False,
                        'use_gst_timestamps': False,
                        'frame_id': frame_id,
                        'camera_name': camera_name,
                        'camera_info_url': camera_config,  # Camera calibration information
                    },
        ],
    )

    # TF publisher
    tf_arg = DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Publish TF?')
    tf_pub_node = Node(package = "tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0.3", "0", "0", "0", "map", frame_id],
                condition=IfCondition(LaunchConfiguration("publish_tf"))
                )

    # Rviz2
    rviz_arg = DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='Launch Rviz?')
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('gmsl_ros2'), 'rviz', 'image_view.rviz')],
            condition=IfCondition(LaunchConfiguration("open_rviz"))
            )

    return LaunchDescription([camera_node, tf_arg, tf_pub_node, rviz_arg, rviz_node])
