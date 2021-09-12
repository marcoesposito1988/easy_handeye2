from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg_eye_in_hand = DeclareLaunchArgument('eye_in_hand')
    arg_tracking_base_frame = DeclareLaunchArgument('tracking_base_frame')
    arg_tracking_marker_frame = DeclareLaunchArgument('tracking_marker_frame')
    arg_robot_base_frame = DeclareLaunchArgument('robot_base_frame')
    arg_robot_effector_frame = DeclareLaunchArgument('robot_effector_frame')

    node_dummy_calib_eih = Node(package='tf2_ros', executable='static_transform_publisher', name='dummy_publisher',
                            condition=IfCondition(LaunchConfiguration('eye_in_hand')),
                            arguments=f'0 0 0.1 0 0 0 1'.split(' ') + [LaunchConfiguration('robot_effector_frame'), LaunchConfiguration('tracking_base_frame')])

    node_dummy_calib_eob = Node(package='tf2_ros', executable='static_transform_publisher', name='dummy_publisher',
                            condition=UnlessCondition(LaunchConfiguration('eye_in_hand')),
                            arguments=f'1 0 0 0 0 0 1'.split(' ') + [LaunchConfiguration('robot_base_frame'), LaunchConfiguration('tracking_base_frame')])

    return LaunchDescription([
        arg_eye_in_hand,
        arg_tracking_base_frame,
        arg_tracking_marker_frame,
        arg_robot_base_frame,
        arg_robot_effector_frame,
        node_dummy_calib_eih,
        node_dummy_calib_eob,
    ])
