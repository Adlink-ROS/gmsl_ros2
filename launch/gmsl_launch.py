"""Launch a Node with parameters and remappings."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # Set parameters
    namespace = LaunchConfiguration('namespace', default='camera')
    node_name = LaunchConfiguration('node_name', default='gmsl_camera_node')
    frame_id = LaunchConfiguration('frame_id', default='gmsl_camera_frame')
    camera_name = LaunchConfiguration('camera_name', default='gmsl_camera')
    camera_config = LaunchConfiguration('camera_config', default='file://' + os.path.join(get_package_share_directory('gmsl_ros2'), 'cfg', 'calibration_param_example.yaml'))
    camera_dev = LaunchConfiguration('camera_dev', default='/dev/video0')
    camera_type = LaunchConfiguration('camera_type', default='argus')

    # v4l2 camera node
    v4l2_camera_node = Node(
        package='gmsl_ros2',
        executable='gmsl_main',
        output='screen',
        name=node_name,
        namespace=namespace,
        parameters=[{
                    'gst_config': (['v4l2src device=', camera_dev, ' ! video/x-raw,format=(string)UYVY,framerate=30/1,width=1280,height=720 ! videoconvert ! video/x-raw, format=(string)BGR ! videoconvert']),
                    'preroll': False,
                    'use_gst_timestamps': False,
                    'frame_id': frame_id,
                    'camera_name': camera_name,
                    'camera_info_url': camera_config,  # Camera calibration information
                   }],
        condition=IfCondition(PythonExpression(['"', camera_type, '" == "v4l2"'])),
    )

    # webcam camera node
    webcam_camera_node = Node(
        package='gmsl_ros2',
        executable='gmsl_main',
        output='screen',
        name=node_name,
        namespace=namespace,
        parameters=[{
                    'gst_config': (['v4l2src device=', camera_dev, ' ! videoconvert']),
                    'preroll': False,
                    'use_gst_timestamps': False,
                    'frame_id': frame_id,
                    'camera_name': camera_name,
                    'camera_info_url': camera_config,  # Camera calibration information
                   }],
        condition=IfCondition(PythonExpression(['"', camera_type, '" == "webcam"'])),
    )

    # argus camera node
    argus_camera_node = Node(
        package='gmsl_ros2',
        executable='gmsl_main',
        output='screen',
        name=node_name,
        namespace=namespace,
        parameters=[{
                    'gst_config': (['nvarguscamerasrc sensor-id=', camera_dev, ' ! video/x-raw(memory:NVMM), width=2048, height=1280, framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)I420 ! videoconvert']),
                    'preroll': False,
                    'use_gst_timestamps': False,
                    'frame_id': frame_id,
                    'camera_name': camera_name,
                    'camera_info_url': camera_config,  # Camera calibration information
                   }],
        condition=IfCondition(PythonExpression(['"', camera_type, '" == "argus"'])),
    )

    # TF publisher
    publish_tf = LaunchConfiguration('publish_tf', default='true')
    tf_pub_node = Node(package="tf2_ros",
                       executable="static_transform_publisher",
                       arguments=["0", "0", "0.3", "0",
                                  "0", "0", "map", frame_id],
                       condition=IfCondition(publish_tf)
                       )

    # Rviz2
    open_rviz = LaunchConfiguration('open_rviz', default='false')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(get_package_share_directory('gmsl_ros2'), 'rviz', 'image_view.rviz')],
        condition=IfCondition(open_rviz)
    )

    return LaunchDescription([v4l2_camera_node, 
                              webcam_camera_node,
                              argus_camera_node,
                              tf_pub_node,
                              rviz_node])
