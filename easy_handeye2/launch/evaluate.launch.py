from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg_name = DeclareLaunchArgument('name')

    handeye_rqt_evaluator = Node(package='easy_handeye2', executable='rqt_evaluator.py',
                                  name='handeye_rqt_evaluator',
                                  # arguments=['--ros-args', '--log-level', 'debug'],
                                  parameters=[{
                                      'name': LaunchConfiguration('name'),
                                  }])

    return LaunchDescription([
        arg_name,
        handeye_rqt_evaluator,
    ])
