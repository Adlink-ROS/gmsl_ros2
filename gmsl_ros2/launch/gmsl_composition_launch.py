"""
Launch a ComposableNode with parameters and remappings.

Limitations:
 -- stdout is not set to flush after each line, so the most recent log messages may be delayed
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Set parameters
namespace = 'camera'
frame_id = 'gmsl_camera_frame'
camera_name = 'gmsl_camera'
camera_config = 'file://' + os.path.join(get_package_share_directory('gmsl_ros2'), 'cfg', 'calibration_param_example.yaml')

def generate_launch_description():

    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gmsl_ros2',
                plugin='gscam2::GSCamNode',
                name='gmsl_publisher',
                namespace=namespace,
                parameters=[
                    {
                        'gst_config': 'v4l2src device=/dev/video0 ! videoconvert',
                        'preroll': False,
                        'use_gst_timestamps': False,
                        'frame_id': frame_id,
                        'camera_name': camera_name,
                        'camera_info_url': camera_config,  # Camera calibration information
                    },
                ],
                # Remap outputs to the correct namespace
                remappings=[
                    ('/image_raw', '/' + namespace + '/image_raw'),
                    ('/camera_info', '/' + namespace + '/camera_info'),
                ],
                extra_arguments=[{
                    'use_intra_process_comms': True,
                }],
            ),
        ],
        output='screen',
    )

    # TF publisher
    tf_pub_node = Node(package = "tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0.3", "0", "0", "0", "map", frame_id])

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

    return LaunchDescription([container, tf_pub_node, rviz_arg, rviz_node])
