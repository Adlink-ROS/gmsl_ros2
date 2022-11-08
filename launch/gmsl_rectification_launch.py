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
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Set parameters
namespace = LaunchConfiguration('namespace', default='camera')
frame_id = LaunchConfiguration('frame_id', default='gmsl_camera_frame')
camera_name = LaunchConfiguration('camera_name', default='gmsl_camera')

# width = LaunchConfiguration('width', default='2048')
# height = LaunchConfiguration('height', default='1280')
width = LaunchConfiguration('width', default='1280')
height = LaunchConfiguration('height', default='720')
# width = LaunchConfiguration('width', default='640')
# height = LaunchConfiguration('height', default='480')

camera_config = LaunchConfiguration('camera_config', default=[
            TextSubstitution(text='file://' + os.path.join(get_package_share_directory('gmsl_ros2'), 'cfg', '')),
            TextSubstitution(text='calibration_param_example_'),
            width,
            TextSubstitution(text='x'),
            height,
            TextSubstitution(text='.yaml')])

# camera index
index = LaunchConfiguration('index', default='0')
camera_dev = LaunchConfiguration('camera_dev', default='/dev/video0')
camera_type = LaunchConfiguration('camera_type', default='argus')
open_rviz = LaunchConfiguration('open_rviz', default='false')
publish_tf = LaunchConfiguration('publish_tf', default='true')

def generate_launch_description():

    image_proc_composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer_node',
            namespace=namespace,
            # Remap subscribers and publishers
            remappings=[
                ('image_raw', 'image_raw'),
                ('image_mono', 'image_mono'),
                ('image_color', 'image_color'),
            ],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_mono_node',
            namespace=namespace,
            # Remap subscribers and publishers
            remappings=[
                ('image', 'image_mono'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect')
            ],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_color_node',
            namespace=namespace,
            # Remap subscribers and publishers
            remappings=[
                ('image', 'image_color'),
                ('image_rect', 'image_rect_color'),
                ('camera_info', 'camera_info')
            ],
        )
    ]

    # v4l2 camera node
    container_v4l2 = ComposableNodeContainer(
        condition=IfCondition(PythonExpression(['"', camera_type, '" == "v4l2"'])),
        name='camera_container',
        namespace=namespace,
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
                        'gst_config': (['v4l2src device=', camera_dev, ' ! video/x-raw,format=(string)UYVY,framerate=30/1,width=', width,',height=', height,' ! videoconvert ! video/x-raw, format=(string)BGR ! videoconvert']),
                        'preroll': False,
                        'use_gst_timestamps': False,
                        'frame_id': frame_id,
                        'camera_name': camera_name,
                        'camera_info_url': camera_config,  # Camera calibration information
                    },
                ],
                # Remap outputs to the correct namespace
                # remappings=[
                #     ('/image_raw', '/' + namespace + '/image_raw'),
                #     ('/camera_info', '/' + namespace + '/camera_info'),
                # ],
                extra_arguments=[{
                    'use_intra_process_comms': True,
                }],
            ),
        ] + image_proc_composable_nodes,
        output='screen',
    )

    # argus camera node
    container_argus = ComposableNodeContainer(
        condition=IfCondition(PythonExpression(['"', camera_type, '" == "argus"'])),
        name='camera_container',
        namespace=namespace,
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
                        'gst_config': (['nvarguscamerasrc sensor-id=', index, ' ! video/x-raw(memory:NVMM), format=NV12, width=', width,', height=', height,', framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)UYVY ! videoconvert']),
                        'preroll': False,
                        'use_gst_timestamps': False,
                        'frame_id': frame_id,
                        'camera_name': camera_name,
                        'camera_info_url': camera_config,  # Camera calibration information
                    },
                ],
                # Remap outputs to the correct namespace
                # remappings=[
                #     ('/image_raw', '/' + namespace + '/image_raw'),
                #     ('/camera_info', '/' + namespace + '/camera_info'),
                # ],
                extra_arguments=[{
                    'use_intra_process_comms': True,
                }],
            ),
        ] + image_proc_composable_nodes,
        output='screen',
    )

    # TF publisher
    tf_pub_node = Node(package = "tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0.3", "0", "0", "0", "map", frame_id],
                condition=IfCondition(publish_tf))

    # Rviz2
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('gmsl_ros2'), 'rviz', 'image_rectification.rviz')],
            condition=IfCondition(open_rviz)
            )

    return LaunchDescription([container_v4l2, container_argus, tf_pub_node, rviz_node])
