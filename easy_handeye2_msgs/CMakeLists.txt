cmake_minimum_required(VERSION 3.5)
project(easy_handeye2_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
if("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "iron")
    find_package(service_msgs REQUIRED) # should be automatically added, it doesn't because we have subdirs in srv :\
    set(SERVICE_MSGS_DEPS "service_msgs")
endif()
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        msg/HandeyeCalibration.msg
        msg/HandeyeCalibrationParameters.msg
        msg/Sample.msg
        msg/SampleList.msg
        msg/TargetPoseList.msg
        srv/calibration/ListAlgorithms.srv
        srv/calibration/SetAlgorithm.srv
        srv/calibration/ComputeCalibration.srv
        srv/calibration/SaveCalibration.srv
        srv/calibration/RemoveSample.srv
        srv/calibration/TakeSample.srv
        srv/calibration/SaveSamples.srv
        srv/calibration/LoadSamples.srv
        srv/robot_movements/CheckStartingPose.srv
        srv/robot_movements/EnumerateTargetPoses.srv
        srv/robot_movements/SelectTargetPose.srv
        srv/robot_movements/PlanToSelectedTargetPose.srv
        srv/robot_movements/ExecutePlan.srv
        DEPENDENCIES std_msgs geometry_msgs ${SERVICE_MSGS_DEPS}
)

ament_package()