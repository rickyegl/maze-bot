import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('maze_bot')
    arg = LaunchConfiguration
    teleop = arg('teleop')

    def node(name, condition=None, **params):
        return Node(package='maze_bot', executable=f'{name}.py', output='screen', parameters=[params],
                    condition=condition)

    description = ParameterValue(Command(['xacro ', os.path.join(pkg, 'urdf', 'robot.urdf.xacro')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('strategy', default_value='flood'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('track', default_value='a'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('camera', default_value='/dev/video0'),
        DeclareLaunchArgument('button_gpio', default_value='-1'),
        DeclareLaunchArgument('lcd_address', default_value='0x27'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='sllidar_ros2', executable='sllidar_node', output='screen',
             parameters=[{'serial_port': arg('lidar_port'), 'serial_baudrate': 460800,
                          'frame_id': 'lidar_link', 'angle_compensate': True, 'scan_mode': 'Standard'}],
             remappings=[('scan', 'scan_raw')]),
        Node(package='v4l2_camera', executable='v4l2_camera_node',
             parameters=[{'video_device': arg('camera'), 'image_size': [640, 480], 'time_per_frame': [1, 5]}],
             remappings=[('image_raw', 'camera/image_raw'), ('camera_info', 'camera/camera_info')]),
        node('base'),
        node('wall_map'),
        node('color_sensor', profile='robot', track=arg('track')),
        node('aruco_detector'),
        node('display', backend='lcd', address=ParameterValue(arg('lcd_address'), value_type=int)),
        Node(package='joy', executable='joy_node', condition=IfCondition(teleop)),
        node('joy_drive', condition=IfCondition(teleop)),
        node('explorer', condition=UnlessCondition(teleop), strategy=arg('strategy'),
             button_gpio=ParameterValue(arg('button_gpio'), value_type=int)),
    ])
