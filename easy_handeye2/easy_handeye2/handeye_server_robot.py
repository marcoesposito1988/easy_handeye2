import math

import rclpy
from rclpy.executors import ExternalShutdownException
import easy_handeye2_msgs as ehm
from easy_handeye2_msgs import msg, srv

import easy_handeye2 as hec
from easy_handeye2.handeye_calibration import HandeyeCalibrationParameters
from easy_handeye2.handeye_robot import CalibrationMovements


class HandeyeServerRobot(rclpy.node.Node):
    def __init__(self, namespace=None):
        super().__init__('handeye_server_robot')
        if namespace is None:
            namespace = self.get_namespace()
        if not namespace.endswith('/'):
            namespace = namespace+'/'
        self.parameters = HandeyeCalibrationParameters.read_from_parameter_server(self, namespace)
        self.declare_parameter('rotation_delta_degrees', 25)
        self.declare_parameter('translation_delta_meters', 0.1)
        self.declare_parameter('max_velocity_scaling', 0.5)
        self.declare_parameter('max_acceleration_scaling', 0.5)
        angle_delta = math.radians(self.get_parameter('rotation_delta_degrees').get_parameter_value().double_value)
        translation_delta = self.get_parameter('translation_delta_meters').get_parameter_value().double_value
        max_velocity_scaling = self.get_parameter('max_velocity_scaling').get_parameter_value().double_value
        max_acceleration_scaling = self.get_parameter('max_acceleration_scaling').get_parameter_value().double_value
        self.local_mover = CalibrationMovements(self, move_group_name=self.parameters.move_group,
                                                move_group_namespace=self.parameters.move_group_namespace,
                                                max_velocity_scaling=max_velocity_scaling,
                                                max_acceleration_scaling=max_acceleration_scaling,
                                                angle_delta=angle_delta, translation_delta=translation_delta)

        # setup services and topics

        self.check_starting_position_service = self.create_service(ehm.srv.CheckStartingPose, hec.CHECK_STARTING_POSE_TOPIC, self.check_starting_position)
        self.enumerate_target_poses_service = self.create_service(ehm.srv.EnumerateTargetPoses, hec.ENUMERATE_TARGET_POSES_TOPIC, self.enumerate_target_poses)
        self.select_target_pose_service = self.create_service(ehm.srv.SelectTargetPose, hec.SELECT_TARGET_POSE_TOPIC, self.select_target_pose)
        self.plan_to_selected_target_pose_service = self.create_service(ehm.srv.PlanToSelectedTargetPose, hec.PLAN_TO_SELECTED_TARGET_POSE_TOPIC, self.plan_to_selected_target_pose)
        self.execute_plan_service = self.create_service(ehm.srv.ExecutePlan, hec.EXECUTE_PLAN_TOPIC, self.execute_plan)

    def check_starting_position(self, _):
        can_calibrate = self.local_mover.set_and_check_starting_position()
        target_poses = ehm.msg.TargetPoseList(home_pose=self.local_mover.start_pose,
                                              target_poses=self.local_mover.target_poses,
                                              current_target_pose_index=self.local_mover.current_pose_index)
        return ehm.srv.CheckStartingPoseResponse(can_calibrate=can_calibrate, target_poses=target_poses)

    def enumerate_target_poses(self, _):
        target_poses = ehm.msg.TargetPoseList(home_pose=self.local_mover.start_pose,
                                              target_poses=self.local_mover.target_poses,
                                              current_target_pose_index=self.local_mover.current_pose_index)
        return ehm.srv.EnumerateTargetPosesResponse(target_poses=target_poses)

    def select_target_pose(self, req):
        success = self.local_mover.select_target_pose(req.target_pose_index)
        target_poses = ehm.msg.TargetPoseList(home_pose=self.local_mover.start_pose,
                                              target_poses=self.local_mover.target_poses,
                                              current_target_pose_index=self.local_mover.current_pose_index)
        return ehm.srv.SelectTargetPoseResponse(success=success, target_poses=target_poses)

    def plan_to_selected_target_pose(self, _):
        ret = self.local_mover.plan_to_current_target_pose()
        return ehm.srv.PlanToSelectedTargetPoseResponse(success=ret)

    def execute_plan(self, _):
        ret = self.local_mover.execute_plan()
        return ehm.srv.ExecutePlanResponse(success=ret)


def main(args=None):
    rclpy.init(args=args)

    handeye_server_robot = HandeyeServerRobot()

    try:
        rclpy.spin(handeye_server_robot)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        handeye_server_robot.destroy_node()


if __name__ == '__main__':
    main()
