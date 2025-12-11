import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Launch the aruco tracker
    pkg_aruco_opencv = get_package_share_directory('aruco_opencv')
    aruco_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(pkg_aruco_opencv, 'launch', 'aruco_tracker.launch.xml'),
        )
    )
    
    # Run scripts of the assignment
    marker_detection = Node(
        package="assignment1",
        executable="aruco_detection.py",
        output='screen',
        parameters=[{
            'image_topic': '/camera/image',
            'base_frame': 'base_footprint'
        }]
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(aruco_launch)
    launchDescriptionObject.add_action(marker_detection)    

    return launchDescriptionObject
