"""Launch a Node with parameters and remappings."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes

def generate_launch_description():

    # Composable camera container
    container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='camera_container',
        namespace='',
    )

    # Composable camera nodes
    camera_nodes = LoadComposableNodes(
        target_container='camera_container',
        composable_node_descriptions = [
            ComposableNode(
                package='gmsl_ros2',
                plugin='gscam2::GSCamNode',
                namespace='camera1',
                parameters=[{
                    'gst_config': 'v4l2src device=/dev/video0 ! videoconvert',
                    'preroll': False,
                    'use_gst_timestamps': False,
                    'frame_id': 'gmsl_camera_frame1',
                    'camera_name': 'gmsl_camera1',
                    'camera_info_url': 'file://' + os.path.join(get_package_share_directory('gmsl_ros2'), 'cfg', 'calibration_param_example.yaml'),  # Camera calibration information
                }],
                extra_arguments=[{
                    'use_intra_process_comms': True,
                }],
            ),
            ComposableNode(
                package='gmsl_ros2',
                plugin='gscam2::GSCamNode',
                namespace='camera2',
                parameters=[{
                    'gst_config': 'v4l2src device=/dev/video2 ! videoconvert',
                    'preroll': False,
                    'use_gst_timestamps': False,
                    'frame_id': 'gmsl_camera_frame2',
                    'camera_name': 'gmsl_camera2',
                    'camera_info_url': 'file://' + os.path.join(get_package_share_directory('gmsl_ros2'), 'cfg', 'calibration_param_example.yaml'),  # Camera calibration information
                }],
                extra_arguments=[{
                    'use_intra_process_comms': True,
                }],
            ),
        ],
    )

    # Composable image displayer nodes
    show_image = DeclareLaunchArgument(
        'show_image',
        default_value='false',
        description='Launch Rviz?'
    )
    image_displayers = LoadComposableNodes(
        target_container='camera_container',
        condition=IfCondition(LaunchConfiguration("show_image")),
        composable_node_descriptions = [
            ComposableNode(
                package='image_view',
                plugin='image_view::ImageViewNode',
                name='receiver1',
                remappings=[('/image', '/camera1/image_raw')],
                extra_arguments=[{
                    'use_intra_process_comms': True,
                }],
            ),
            ComposableNode(
                package='image_tools',
                plugin='image_tools::ShowImage',
                name='receiver2',
                remappings=[('/image', '/camera2/image_raw')],
                extra_arguments=[{
                    'use_intra_process_comms': False,
                }],
            ),
        ],
    )

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

    # Rviz2
    rviz_arg = DeclareLaunchArgument(
        'open_rviz',
        default_value='false',
        description='Launch Rviz?'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('gmsl_ros2'), 'rviz', 'multi_camera.rviz')],
        condition=IfCondition(LaunchConfiguration("open_rviz"))
    )

    return LaunchDescription([container, camera_nodes, tf_pub_group_node, show_image, image_displayers, rviz_arg, rviz_node])
