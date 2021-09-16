import itertools

import easy_handeye2_msgs as ehm
import rclpy
import std_msgs
import std_srvs
from easy_handeye2_msgs import msg, srv
from std_msgs import msg
from std_srvs import srv

import easy_handeye2 as hec
from easy_handeye2.handeye_calibration import HandeyeCalibration, HandeyeCalibrationParameters, \
    HandeyeCalibrationParametersProvider
from easy_handeye2.handeye_calibration_backend_opencv import HandeyeCalibrationBackendOpenCV
from easy_handeye2.handeye_sampler import HandeyeSampler


class HandeyeServer(rclpy.node.Node):
    def __init__(self):
        super().__init__('handeye_server')

        self.parameters_provider = HandeyeCalibrationParametersProvider(self)
        self.parameters = self.parameters_provider.read()

        self.get_logger().info(f'Read parameters for calibration "{self.parameters.name}"')

        self.sampler = HandeyeSampler(self, handeye_parameters=self.parameters)
        self.calibration_backends = {'OpenCV': HandeyeCalibrationBackendOpenCV()}
        self.calibration_algorithm = 'OpenCV/Tsai-Lenz'

        # setup calibration services and topics

        self.list_algorithms_service = self.create_service(ehm.srv.ListAlgorithms, hec.LIST_ALGORITHMS_TOPIC,
                                                           self.list_algorithms)
        self.set_algorithm_service = self.create_service(ehm.srv.SetAlgorithm, hec.SET_ALGORITHM_TOPIC,
                                                         self.set_algorithm)
        self.get_sample_list_service = self.create_service(ehm.srv.TakeSample, hec.GET_SAMPLE_LIST_TOPIC,
                                                           self.get_sample_lists)
        self.take_sample_service = self.create_service(ehm.srv.TakeSample, hec.TAKE_SAMPLE_TOPIC, self.take_sample)
        self.remove_sample_service = self.create_service(ehm.srv.RemoveSample, hec.REMOVE_SAMPLE_TOPIC,
                                                         self.remove_sample)
        self.compute_calibration_service = self.create_service(ehm.srv.ComputeCalibration,
                                                               hec.COMPUTE_CALIBRATION_TOPIC, self.compute_calibration)
        self.save_calibration_service = self.create_service(std_srvs.srv.Empty, hec.SAVE_CALIBRATION_TOPIC,
                                                            self.save_calibration)

        # Useful for secondary input sources (e.g. programmable buttons on robot)
        self.take_sample_topic = self.create_subscription(std_msgs.msg.Empty, hec.TAKE_SAMPLE_TOPIC, self.take_sample,
                                                          10)
        self.compute_calibration_topic = self.create_subscription(std_msgs.msg.Empty, hec.REMOVE_SAMPLE_TOPIC,
                                                                  self.remove_last_sample, 10)

        self.last_calibration = None

    # algorithm

    def list_algorithms(self, _, response: ehm.srv.ListAlgorithms.Response):
        algorithms_nested = [[bck_name + '/' + alg_name for alg_name in bck.AVAILABLE_ALGORITHMS] for bck_name, bck in
                             self.calibration_backends.items()]
        available_algorithms = list(itertools.chain(*algorithms_nested))
        response.algorithms = available_algorithms
        response.current_algorithm = self.calibration_algorithm
        return response

    def set_algorithm(self, req: ehm.srv.SetAlgorithm.Request, response: ehm.srv.SetAlgorithm.Response):
        alg_to_set = req.new_algorithm
        bckname_algname = alg_to_set.split('/')
        if len(bckname_algname) != 2:
            response.success = False
            return response
        bckname, algname = bckname_algname
        if bckname not in self.calibration_backends:
            response.success = False
            return response
        if algname not in self.calibration_backends[bckname].AVAILABLE_ALGORITHMS:
            response.success = False
            return response
        self.get_logger().info('switching to calibration algorithm {}'.format(alg_to_set))
        self.calibration_algorithm = alg_to_set
        response.success = True
        return response

    # sampling

    def _retrieve_sample_list(self):
        ret = ehm.msg.SampleList()
        for s in self.sampler.get_samples():
            ret.camera_marker_samples.append(s['optical'].transform)
            ret.hand_world_samples.append(s['robot'].transform)
        return ret

    def get_sample_lists(self, _, response: ehm.srv.TakeSample.Response):
        response.samples = self._retrieve_sample_list()
        return response

    def take_sample(self, _=None, response: ehm.srv.TakeSample.Response = None):
        self.sampler.take_sample()
        if response is not None:
            # if it is the service, not the topic
            response.samples = self._retrieve_sample_list()
            return response

    def remove_last_sample(self):
        self.sampler.remove_sample(len(self.sampler.samples) - 1)

    def remove_sample(self, req: ehm.srv.RemoveSample.Request, response: ehm.srv.RemoveSample.Response):
        try:
            self.sampler.remove_sample(req.sample_index)
        except IndexError:
            self.get_logger().err('Invalid index ' + req.sample_index)
        response.samples = self._retrieve_sample_list()
        return response

    # calibration

    def compute_calibration(self, _, response: ehm.srv.ComputeCalibration.Response):
        samples = self.sampler.get_samples()

        bckname, algname = self.calibration_algorithm.split('/')
        backend = self.calibration_backends[bckname]

        self.last_calibration = backend.compute_calibration(self.parameters, samples, algorithm=algname)
        if self.last_calibration is None:
            self.get_logger().warn('No valid calibration computed')
            response.valid = False
            return response
        response.valid = True
        response.calibration.eye_in_hand = self.last_calibration.parameters.eye_in_hand
        response.calibration.transform = self.last_calibration.transformation
        return response

    def save_calibration(self, _, response: std_srvs.srv.Empty.Response):
        if self.last_calibration:
            HandeyeCalibration.to_file(self.last_calibration)
            self.get_logger().info('Calibration saved to {}'.format(self.last_calibration.filename()))
        return response

    # TODO: evaluation


def main(args=None):
    rclpy.init(args=args)

    handeye_server = HandeyeServer()

    rclpy.spin(handeye_server)

    handeye_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
