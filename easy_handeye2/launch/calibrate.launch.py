from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    arg_name = DeclareLaunchArgument('name')
    arg_eye_in_hand = DeclareLaunchArgument('eye_in_hand')
    arg_tracking_base_frame = DeclareLaunchArgument('tracking_base_frame')
    arg_tracking_marker_frame = DeclareLaunchArgument('tracking_marker_frame')
    arg_robot_base_frame = DeclareLaunchArgument('robot_base_frame')
    arg_robot_effector_frame = DeclareLaunchArgument('robot_effector_frame')

    node_dummy_calib_eih = Node(package='tf2_ros', executable='static_transform_publisher', name='dummy_publisher',
                                condition=IfCondition(LaunchConfiguration('eye_in_hand')),
                                arguments=f'0 0 0.1 0 0 0 1'.split(' ') + [LaunchConfiguration('robot_effector_frame'),
                                                                           LaunchConfiguration('tracking_base_frame')])

    node_dummy_calib_eob = Node(package='tf2_ros', executable='static_transform_publisher', name='dummy_publisher',
                                condition=UnlessCondition(LaunchConfiguration('eye_in_hand')),
                                arguments=f'1 0 0 0 0 0 1'.split(' ') + [LaunchConfiguration('robot_base_frame'),
                                                                         LaunchConfiguration('tracking_base_frame')])

    handeye_server = Node(package='easy_handeye2', executable='handeye_server', name='handeye_server', parameters=[{
        'name': LaunchConfiguration('name'),
        'eye_in_hand': LaunchConfiguration('eye_in_hand'),
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
            'eye_in_hand': LaunchConfiguration('eye_in_hand'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
        }])

    return LaunchDescription([
        arg_name,
        arg_eye_in_hand,
        arg_tracking_base_frame,
        arg_tracking_marker_frame,
        arg_robot_base_frame,
        arg_robot_effector_frame,
        node_dummy_calib_eih,
        node_dummy_calib_eob,
        handeye_server,
        handeye_rqt_calibrator,
    ])
