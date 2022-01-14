"""Launch a Node with parameters and remappings."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Rviz2, these codes should be placed before camera nodes
    rviz_arg = DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='Launch Rviz?')
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('gmsl_ros2'), 'rviz', 'multi_camera.rviz')],
            condition=IfCondition(LaunchConfiguration("open_rviz"))
            )

    # Camera nodes
    camera_group_node = GroupAction([
        # Camera 1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gmsl_launch.py']),
            launch_arguments = {
                'namespace': 'camera1',
                'node_name': 'gmsl_camera_node1',
                'frame_id' :'gmsl_camera_frame1',
                'camera_name' : 'gmsl_camera1',
                'camera_config' : 'file://' + os.path.join(get_package_share_directory('gmsl_ros2'), 'cfg', 'calibration_param_example.yaml'),
                'camera_dev' : '/dev/video0',
                'open_rviz' : 'false',
                'publish_tf': 'false',
            }.items()
        ),
        # Camera 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gmsl_launch.py']),
            launch_arguments = {
                'namespace': 'camera2',
                'node_name': 'gmsl_camera_node2',
                'frame_id' :'gmsl_camera_frame2',
                'camera_name' : 'gmsl_camera2',
                'camera_config' : 'file://' + os.path.join(get_package_share_directory('gmsl_ros2'), 'cfg', 'calibration_param_example.yaml'),
                'camera_dev' : '/dev/video2',
                'open_rviz' : 'false',
                'publish_tf': 'false',
            }.items()
        )
    ])

    # TF publisher
    tf_pub_group_node = GroupAction([
        Node(package = "tf2_ros",
            name = "tf_publisher1",
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0.3", "0", "0", "0", "map", "gmsl_camera_frame1"],
        ),
        Node(package = "tf2_ros",
            name = "tf_publisher2",
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0.3", "0", "0", "0", "map", "gmsl_camera_frame2"],
        ),
    ])

    return LaunchDescription([rviz_arg, rviz_node, camera_group_node, tf_pub_group_node])
