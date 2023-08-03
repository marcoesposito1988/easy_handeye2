from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals

arg_name = DeclareLaunchArgument('name')
arg_calibration_type = DeclareLaunchArgument('calibration_type', choices=["eye_in_hand", "eye_on_base"],
                                             description="Type of the calibration")
arg_tracking_base_frame = DeclareLaunchArgument('tracking_base_frame')
arg_tracking_marker_frame = DeclareLaunchArgument('tracking_marker_frame')
arg_robot_base_frame = DeclareLaunchArgument('robot_base_frame')
arg_robot_effector_frame = DeclareLaunchArgument('robot_effector_frame')

args_calibration = [
    arg_name,
    arg_calibration_type,
    arg_tracking_base_frame,
    arg_tracking_marker_frame,
    arg_robot_base_frame,
    arg_robot_effector_frame,
]

is_eye_in_hand = LaunchConfigurationEquals('calibration_type', 'eye_in_hand')

arg_automatic_robot_movement = DeclareLaunchArgument('automatic_robot_movement', default_value='False')
arg_move_group = DeclareLaunchArgument('move_group')
arg_move_group_namespace = DeclareLaunchArgument('move_group_namespace')
arg_translation_delta_meters = DeclareLaunchArgument('translation_delta_meters', default_value='0.1')
arg_rotation_delta_degrees = DeclareLaunchArgument('rotation_delta_degrees', default_value='25')
arg_max_velocity_scaling = DeclareLaunchArgument('max_velocity_scaling', default_value='0.3')
arg_max_acceleration_scaling = DeclareLaunchArgument('max_acceleration_scaling', default_value='0.2')

args_robot = [
    arg_automatic_robot_movement,
    arg_move_group,
    arg_move_group_namespace,
    arg_rotation_delta_degrees,
    arg_translation_delta_meters,
    arg_max_velocity_scaling,
    arg_max_acceleration_scaling,
]
