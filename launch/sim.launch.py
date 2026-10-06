import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('maze_bot')
    world = os.path.join(pkg, 'worlds', 'maze.sdf')
    arg = LaunchConfiguration
    sim = {'use_sim_time': True}

    def node(name, **params):
        return Node(package='maze_bot', executable=f'{name}.py', output='screen', parameters=[{**sim, **params}])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{world} -r -v 2', 'on_exit_shutdown': 'true'}.items())

    description = ParameterValue(Command(['xacro ', os.path.join(pkg, 'urdf', 'robot.urdf.xacro')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('strategy', default_value='flood'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.6'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{**sim, 'robot_description': description}]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-topic', 'robot_description', '-name', 'maze_bot',
                        '-x', arg('x'), '-y', arg('y'), '-z', '0.01', '-Y', arg('yaw')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{**sim, 'config_file': os.path.join(pkg, 'config', 'bridge.yaml')}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(pkg, 'rviz', 'maze.rviz')],
             parameters=[sim], condition=IfCondition(arg('rviz'))),
        node('base'),
        node('wall_map', deskew=False),
        node('color_sensor'),
        node('aruco_detector'),
        node('display', backend='log'),
        node('explorer', strategy=arg('strategy')),
    ])
