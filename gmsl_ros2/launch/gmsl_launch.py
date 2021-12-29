"""Launch a Node with parameters and remappings."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Set parameters
namespace = 'camera'
frame_id = 'gmsl_camera_frame'
camera_name = 'gmsl_camera'
camera_config = 'file://' + os.path.join(get_package_share_directory('gscam2'), 'cfg', 'calibration_param_example.yaml')

def generate_launch_description():

    camera_node = Node(
        package='gscam2',
        executable='gscam_main',
        output='screen',
        name='gscam_publisher',
        namespace=namespace,
        parameters=[
                    {
                        'gscam_config': 'v4l2src device=/dev/video0 ! videoconvert',
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
            arguments=['-d', os.path.join(get_package_share_directory('gscam2'), 'rviz', 'image_view.rviz')],
            condition=IfCondition(LaunchConfiguration("open_rviz"))
            )

    return LaunchDescription([camera_node, tf_pub_node, rviz_arg, rviz_node])
