from easy_handeye2.common_launch import args_calibration, args_robot
from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    node_dummy_calib_eih = Node(package='tf2_ros', executable='static_transform_publisher', name='dummy_publisher',
                                condition=LaunchConfigurationEquals('calibration_type', 'eye_in_hand'),
                                arguments=f'--x 0 --y 0 --z 0.1 --qx 0 --qy 0 --qz 0 --qw 1'.split(' ') + ['--frame-id',
                                                                                                           LaunchConfiguration(
                                                                                                               'robot_effector_frame'),
                                                                                                           '--child-frame-id',
                                                                                                           LaunchConfiguration(
                                                                                                               'tracking_base_frame')])

    node_dummy_calib_eob = Node(package='tf2_ros', executable='static_transform_publisher', name='dummy_publisher',
                                condition=LaunchConfigurationEquals('calibration_type', 'eye_on_base'),
                                arguments=f'--x 1 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1'.split(' ') + ['--frame-id',
                                                                                                         LaunchConfiguration(
                                                                                                             'robot_base_frame'),
                                                                                                         '--child-frame-id',
                                                                                                         LaunchConfiguration(
                                                                                                             'tracking_base_frame')])

    handeye_server = Node(package='easy_handeye2', executable='handeye_server', name='handeye_server', parameters=[{
        'name': LaunchConfiguration('name'),
        'calibration_type': LaunchConfiguration('calibration_type'),
        'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
        'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
        'robot_base_frame': LaunchConfiguration('robot_base_frame'),
        'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
    }])

    handeye_rqt_calibrator = Node(package='easy_handeye2', executable='rqt_handeye_calibrator',
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

    handeye_server_robot = Node(package='easy_handeye2', executable='handeye_server_robot',
                                name='handeye_server_robot',
                                condition=IfCondition(LaunchConfiguration('automatic_robot_movement')),
                                parameters=[{
                                    'name': LaunchConfiguration('name'),
                                    'automatic_robot_movement': LaunchConfiguration('automatic_robot_movement'),
                                    'move_group': LaunchConfiguration('move_group'),
                                    'move_group_namespace': LaunchConfiguration('move_group_namespace'),
                                    'rotation_delta_degrees': LaunchConfiguration('rotation_delta_degrees'),
                                    'translation_delta_meters': LaunchConfiguration('translation_delta_meters'),
                                    'max_velocity_scaling': LaunchConfiguration('max_velocity_scaling'),
                                    'max_acceleration_scaling': LaunchConfiguration('max_acceleration_scaling'),
                                }])

    handeye_rqt_mover = Node(package='easy_handeye2', executable='rqt_handeye_mover',
                                  name='rqt_handeye_mover',
                                  # arguments=['--ros-args', '--log-level', 'debug'],
                                  parameters=[{
                                  }])

    return LaunchDescription(
        args_calibration + args_robot + [
            node_dummy_calib_eih,
            node_dummy_calib_eob,
            handeye_server,
            handeye_rqt_calibrator,
            handeye_server_robot,
            handeye_rqt_mover,
        ])
