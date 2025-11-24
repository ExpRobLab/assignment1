import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():

    # World spawn
    world_arg = DeclareLaunchArgument(
        'world', default_value='my_world_assignment.sdf',
        description='Name of the Gazebo world file to load'
    )

    pkg_world = get_package_share_directory('worlds_manager')
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_world, 'launch', 'my_launch_assignment.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        }.items()
    )

    pkg_robot = get_package_share_directory('robot_manager')
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'spawn_robot_assignment.launch.py'),
        )
        # ,
        # launch_arguments={
        # 'x_arg': 0,
        # }.items()
    )

    # TODO aruco_tracker.launch
    # pkg_aruco_opencv = get_package_share_directory('aruco_opencv')
    # aruco_launch = IncludeLaunchDescription(
    #     XMLLaunchDescriptionSource(
    #         os.path.join(pkg_aruco_opencv, 'launch', 'aruco_tracker.launch.xml'),
    #     )
    # )
    
    # TODO run scripts of the assignment
    
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(robot_launch)
    #launchDescriptionObject.add_action(aruco_launch)    
    return launchDescriptionObject
