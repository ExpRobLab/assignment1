import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    remap_camera = TimerAction(
        period=2.0,
        actions=[Node(
        package='topic_tools',
        executable='relay',
        name='oak_color_relay',
        arguments=['/camera/rgb/camera_info', '/camera/rgb/image_raw/camera_info'],
        output='screen'
        )]
    )
    
    # Launch the aruco tracker
    pkg_aruco_opencv = get_package_share_directory('aruco_opencv')
    aruco_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(pkg_aruco_opencv, 'launch', 'aruco_tracker_husarion.launch.xml'),
        )
    )
    
    # Run scripts of the assignment
    marker_detection = Node(
        package="assignment1",
        executable="aruco_detection.py",
        output='screen',
        parameters=[{
            'image_topic': '/camera/rgb/image_raw',
            'base_frame': 'base_link'
        }]
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(remap_camera)
    launchDescriptionObject.add_action(aruco_launch)
    launchDescriptionObject.add_action(marker_detection)    

    return launchDescriptionObject
