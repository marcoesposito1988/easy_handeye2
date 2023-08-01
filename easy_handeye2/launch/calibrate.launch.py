from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from easy_handeye2.common_launch import arg_calibration_type, arg_tracking_base_frame, arg_tracking_marker_frame, arg_robot_base_frame, \
    arg_robot_effector_frame


def generate_launch_description():
    arg_name = DeclareLaunchArgument('name')

    node_dummy_calib_eih = Node(package='tf2_ros', executable='static_transform_publisher', name='dummy_publisher',
                                condition=LaunchConfigurationEquals('calibration_type', 'eye_in_hand'),
                                arguments=f'--x 0 --y 0 --z 0.1 --qx 0 --qy 0 --qz 0 --qw 1'.split(' ') + ['--frame-id', LaunchConfiguration('robot_effector_frame'),
                                                                                                           '--child-frame-id', LaunchConfiguration('tracking_base_frame')])

    node_dummy_calib_eob = Node(package='tf2_ros', executable='static_transform_publisher', name='dummy_publisher',
                                condition=LaunchConfigurationEquals('calibration_type', 'eye_on_base'),
                                arguments=f'--x 1 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1'.split(' ') + ['--frame-id', LaunchConfiguration('robot_base_frame'),
                                                                                                         '--child-frame-id', LaunchConfiguration('tracking_base_frame')])

    handeye_server = Node(package='easy_handeye2', executable='handeye_server', name='handeye_server', parameters=[{
        'name': LaunchConfiguration('name'),
        'calibration_type': LaunchConfiguration('calibration_type'),
        'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
        'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
        'robot_base_frame': LaunchConfiguration('robot_base_frame'),
        'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
    }])

    handeye_rqt_calibrator = Node(package='easy_handeye2', executable='rqt_calibrator.py',
                                  name='handeye_rqt_calibrator',
                                  # arguments=['--ros-args', '--log-level', 'debug'],
                                  parameters=[{
                                      'name': LaunchConfiguration('name'),
                                      'calibration_type': LaunchConfiguration('calibration_type'),
                                      'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
                                      'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
                                      'robot_base_frame': LaunchConfiguration('robot_base_frame'),
                                      'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
                                  }])

    return LaunchDescription([
        arg_name,
        arg_calibration_type,
        arg_tracking_base_frame,
        arg_tracking_marker_frame,
        arg_robot_base_frame,
        arg_robot_effector_frame,
        node_dummy_calib_eih,
        node_dummy_calib_eob,
        handeye_server,
        handeye_rqt_calibrator,
    ])
