import rclpy
import std_srvs
from std_srvs import srv
import easy_handeye2 as hec
from easy_handeye2.handeye_calibration import HandeyeCalibrationParameters
import easy_handeye2_msgs as ehm
from easy_handeye2_msgs import srv


class HandeyeClient:
    def __init__(self, node: rclpy.node.Node, parameters: HandeyeCalibrationParameters):
        self.node = node
        self.parameters = parameters

        self.node.get_logger().info(f'Configuring HandeyeClient for calibration with namespace: {parameters.name}')

        # init services: sampling
        self.node.get_logger().info('Waiting for sampling services')
        self.get_sample_client = self.node.create_client(ehm.srv.TakeSample, hec.GET_SAMPLE_LIST_TOPIC)
        self.get_sample_client.wait_for_service()
        self.get_current_transforms_client = self.node.create_client(ehm.srv.TakeSample, hec.GET_CURRENT_TRANSFORMS_TOPIC)
        self.get_current_transforms_client.wait_for_service()
        self.take_sample_client = self.node.create_client(ehm.srv.TakeSample, hec.TAKE_SAMPLE_TOPIC)
        self.take_sample_client.wait_for_service()
        self.remove_sample_client = self.node.create_client(ehm.srv.RemoveSample, hec.REMOVE_SAMPLE_TOPIC)
        self.remove_sample_client.wait_for_service()

        # init services: calibration
        self.node.get_logger().info('Waiting for calibration services')
        self.list_algorithms_client = self.node.create_client(ehm.srv.ListAlgorithms, hec.LIST_ALGORITHMS_TOPIC)
        self.list_algorithms_client.wait_for_service()
        self.set_algorithm_client = self.node.create_client(ehm.srv.SetAlgorithm, hec.SET_ALGORITHM_TOPIC)
        self.set_algorithm_client.wait_for_service()
        self.compute_calibration_client = self.node.create_client(ehm.srv.ComputeCalibration,
                                                                  hec.COMPUTE_CALIBRATION_TOPIC)
        self.compute_calibration_client.wait_for_service()
        self.save_calibration_client = self.node.create_client(ehm.srv.SaveCalibration, hec.SAVE_CALIBRATION_TOPIC)
        self.save_calibration_client.wait_for_service()

        if False:  # TODO
        # if not self.parameters.freehand_robot_movement:
            # init services: robot movement
            self.node.get_logger().info('Waiting for robot motion services')
            self.check_starting_pose_client = self.node.create_client(ehm.srv.CheckStartingPose,
                                                                      hec.CHECK_STARTING_POSE_TOPIC)
            self.check_starting_pose_client.wait_for_service()

            self.enumerate_target_poses_client = self.node.create_client(ehm.srv.EnumerateTargetPoses,
                                                                         hec.ENUMERATE_TARGET_POSES_TOPIC)
            self.enumerate_target_poses_client.wait_for_service()

            self.select_target_pose_client = self.node.create_client(ehm.srv.SelectTargetPose,
                                                                     hec.SELECT_TARGET_POSE_TOPIC)
            self.select_target_pose_client.wait_for_service()

            self.plan_to_selected_target_pose_client = self.node.create_client(ehm.srv.PlanToSelectedTargetPose,
                                                                               hec.PLAN_TO_SELECTED_TARGET_POSE_TOPIC)
            self.plan_to_selected_target_pose_client.wait_for_service()

            self.execute_plan_client = self.node.create_client(ehm.srv.ExecutePlan, hec.EXECUTE_PLAN_TOPIC)
            self.execute_plan_client.wait_for_service()

        self.node.get_logger().info('All services connected, ready to go!')

    # services: sampling

    def get_current_transforms(self):
        samples = self.get_current_transforms_client.call(ehm.srv.TakeSample.Request()).samples
        if samples.samples:
            return samples.samples[0]
        return None

    def get_sample_list(self):
        return self.get_sample_client.call(ehm.srv.TakeSample.Request()).samples

    def take_sample(self):
        return self.take_sample_client.call(ehm.srv.TakeSample.Request()).samples

    def remove_sample(self, index):
        return self.remove_sample_client.call(ehm.srv.RemoveSample.Request(sample_index=index)).samples

    # services: calibration

    def list_algorithms(self):
        return self.list_algorithms_client.call(ehm.srv.ListAlgorithms.Request())

    def set_algorithm(self, new_algorithm):
        return self.set_algorithm_client.call(ehm.srv.SetAlgorithm.Request(new_algorithm=new_algorithm))

    def compute_calibration(self):
        return self.compute_calibration_client.call(ehm.srv.ComputeCalibration.Request())

    def save(self):
        return self.save_calibration_client.call(ehm.srv.SaveCalibration.Request())

    # TODO: services: evaluation

    # services: robot movement
    def check_starting_pose(self):
        return self.check_starting_pose_client()

    def enumerate_target_poses(self):
        return self.enumerate_target_poses_client()

    def select_target_pose(self, i):
        return self.select_target_pose_client(i)

    def plan_to_selected_target_pose(self):
        return self.plan_to_selected_target_pose_client()

    def execute_plan(self):
        return self.execute_plan_client()
