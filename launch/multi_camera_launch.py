"""Launch a Node with parameters and remappings."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    open_rviz = LaunchConfiguration('open_rviz', default='false')
    # width = LaunchConfiguration('width', default='2048')
    # height = LaunchConfiguration('height', default='1280')
    width = LaunchConfiguration('width', default='640')
    height = LaunchConfiguration('height', default='480')    
    NUM_OF_CAMS = 8
    
    ld = LaunchDescription()

    # Rviz2
    ld.add_entity(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('gmsl_ros2'), 'rviz', 'multi_camera.rviz')],
            condition=IfCondition(open_rviz)
            )
    )

    # Invoke multiple camera nodes
    for i in range(NUM_OF_CAMS):
        ld.add_action(
            TimerAction(
                period=2.0*int(f'{i}'), # delay 2 seconds between each cameras
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gmsl_launch.py']),
                        launch_arguments = {
                            'index' : f'{i}',
                            'namespace': f'camera{i}',
                            'node_name': f'gmsl_camera_node{i}',
                            'frame_id' : f'gmsl_camera_frame{i}',
                            'camera_name' : f'gmsl_camera{i}',
                            'camera_config' : 'file://' + os.path.join(get_package_share_directory('gmsl_ros2'), 'cfg', 'calibration_param_example.yaml'),
                            'camera_dev' : f'/dev/video{i}',
                            'width' : width,
                            'height' : height,
                            'open_rviz' : 'false',
                            'publish_tf': 'false',
                        }.items()
                    )
                ]
            )
        )

        # TF publisher
        ld.add_entity(
            Node(
                package="tf2_ros",
                executable=f"static_transform_publisher",
                name=f"static_transform_publisher{i}",
                arguments=["0", f"0.{i}", "0.3", "0", "0", "0", "map", f"gmsl_camera_frame{i}"],
                )
        )


    return ld
