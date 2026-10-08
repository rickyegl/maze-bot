from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    def arg(name, kind):
        return ParameterValue(LaunchConfiguration(name), value_type=kind)

    test = Node(package='maze_bot', executable='drive_test.py', output='screen',
                parameters=[{'speed': arg('speed', float), 'seconds': arg('seconds', float),
                             'hold': arg('hold', bool), 'square': arg('square', bool),
                             'side': arg('side', float), 'spin': arg('spin', float)}])
    return LaunchDescription([
        DeclareLaunchArgument('speed', default_value='0.15'),
        DeclareLaunchArgument('seconds', default_value='6.0'),
        DeclareLaunchArgument('hold', default_value='true'),
        DeclareLaunchArgument('square', default_value='false'),
        DeclareLaunchArgument('side', default_value='0.5'),
        DeclareLaunchArgument('spin', default_value='0.0'),
        Node(package='maze_bot', executable='imu.py', output='screen'),
        Node(package='maze_bot', executable='base.py', output='screen'),
        Node(package='maze_bot', executable='motors.py', output='screen'),
        test,
        RegisterEventHandler(OnProcessExit(target_action=test, on_exit=[EmitEvent(event=Shutdown())])),
    ])
