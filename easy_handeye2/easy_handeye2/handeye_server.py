import itertools

import rclpy
from rclpy.executors import MultiThreadedExecutor
import std_msgs
import easy_handeye2_msgs as ehm
from easy_handeye2_msgs import msg, srv
from rclpy.executors import ExternalShutdownException
from std_msgs import msg

import easy_handeye2 as hec
from easy_handeye2.handeye_calibration import save_calibration, HandeyeCalibrationParametersProvider
from easy_handeye2.handeye_calibration_backend_opencv import HandeyeCalibrationBackendOpenCV
from easy_handeye2.handeye_sampler import HandeyeSampler


class HandeyeServer(rclpy.node.Node):
    def __init__(self):
        super().__init__('handeye_server')

        self.parameters_provider = HandeyeCalibrationParametersProvider(self)
        self.parameters = self.parameters_provider.read()

        self.get_logger().info(f'Read parameters for calibration "{self.parameters.name}"')

        self.sampler = HandeyeSampler(self, handeye_parameters=self.parameters)
        self.setup_timer = self.create_timer(2.0, self.setup_services_and_topics)

        self.calibration_backends = {'OpenCV': HandeyeCalibrationBackendOpenCV()}
        self.calibration_algorithm = 'OpenCV/Tsai-Lenz'

        # setup calibration services and topics
        self.list_algorithms_service = None
        self.set_algorithm_service = None
        self.get_current_transforms_service = None
        self.get_sample_list_service = None
        self.take_sample_service = None
        self.remove_sample_service = None
        self.save_samples_service = None
        self.load_samples_service = None
        self.compute_calibration_service = None
        self.save_calibration_service = None
        self.take_sample_topic = None
        self.remove_last_sample_topic = None

        self.last_calibration = None

    def setup_services_and_topics(self):
        if not self.sampler.wait_for_tf_init():
            self.get_logger().warn('Waiting for TF initialization...')
            return

        self.list_algorithms_service = self.create_service(ehm.srv.ListAlgorithms, hec.LIST_ALGORITHMS_TOPIC,
                                                           self.list_algorithms)
        self.set_algorithm_service = self.create_service(ehm.srv.SetAlgorithm, hec.SET_ALGORITHM_TOPIC,
                                                         self.set_algorithm)
        self.get_current_transforms_service = self.create_service(ehm.srv.TakeSample, hec.GET_CURRENT_TRANSFORMS_TOPIC,
                                                           self.get_current_transforms)
        self.get_sample_list_service = self.create_service(ehm.srv.TakeSample, hec.GET_SAMPLE_LIST_TOPIC,
                                                           self.get_sample_lists)
        self.take_sample_service = self.create_service(ehm.srv.TakeSample, hec.TAKE_SAMPLE_TOPIC, self.take_sample_srv_callback)
        self.remove_sample_service = self.create_service(ehm.srv.RemoveSample, hec.REMOVE_SAMPLE_TOPIC,
                                                         self.remove_sample_srv_callback)
        self.save_samples_service = self.create_service(ehm.srv.SaveSamples, hec.SAVE_SAMPLES_TOPIC,
                                                        self.save_samples)
        self.load_samples_service = self.create_service(ehm.srv.LoadSamples, hec.LOAD_SAMPLES_TOPIC,
                                                        self.load_samples)
        self.compute_calibration_service = self.create_service(ehm.srv.ComputeCalibration,
                                                               hec.COMPUTE_CALIBRATION_TOPIC, self.compute_calibration)
        self.save_calibration_service = self.create_service(ehm.srv.SaveCalibration, hec.SAVE_CALIBRATION_TOPIC,
                                                            self.save_calibration)

        # Useful for secondary input sources (e.g. programmable buttons on robot)
        self.take_sample_topic = self.create_subscription(std_msgs.msg.Empty, hec.TAKE_SAMPLE_TOPIC, self.take_sample_msg_callback,
                                                          10)
        self.remove_last_sample_topic = self.create_subscription(std_msgs.msg.Empty, hec.REMOVE_SAMPLE_TOPIC,
                                                                  self.remove_last_sample, 10)
        self.setup_timer.cancel()

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
        return self.sampler.get_samples()

    def get_current_transforms(self, _, response: ehm.srv.TakeSample.Response):
        transforms = self.sampler.current_transforms()
        if transforms is None:
            response.samples.samples = []
            return response

        response.samples.samples = [transforms]
        return response

    def get_sample_lists(self, _, response: ehm.srv.TakeSample.Response):
        response.samples = self._retrieve_sample_list()
        return response

    def take_sample_srv_callback(self, _, response: ehm.srv.TakeSample.Response):
        self.sampler.take_sample()
        response.samples = self._retrieve_sample_list()
        return response

    def take_sample_msg_callback(self, _):
        self.sampler.take_sample()

    def remove_last_sample(self, _):
        self.sampler.remove_sample(len(self.sampler.samples) - 1)

    def remove_sample_srv_callback(self, req: ehm.srv.RemoveSample.Request, response: ehm.srv.RemoveSample.Response):
        try:
            self.sampler.remove_sample(req.sample_index)
        except IndexError:
            self.get_logger().err('Invalid index ' + req.sample_index)
        response.samples = self._retrieve_sample_list()
        return response

    def save_samples(self, _: ehm.srv.SaveSamples.Request, response: ehm.srv.SaveSamples.Response):
        try:
            response.success = self.sampler.save_samples()
        except Exception as e:
            self.get_logger().error(e)
            response.success = False
        return response

    def load_samples(self, _: ehm.srv.LoadSamples.Request, response: ehm.srv.LoadSamples.Response):
        try:
            response.success = self.sampler.load_samples()
            response.samples = self._retrieve_sample_list()
        except IndexError:
            response.success = False
        return response

    # calibration

    def compute_calibration(self, _, response: ehm.srv.ComputeCalibration.Response):
        samples = self.sampler.get_samples()

        bckname, algname = self.calibration_algorithm.split('/')
        backend = self.calibration_backends[bckname]

        self.last_calibration = backend.compute_calibration(self, self.parameters, samples, algorithm=algname)
        if self.last_calibration is None:
            self.get_logger().warn('No valid calibration computed')
            response.valid = False
            return response
        response.valid = True
        response.calibration = self.last_calibration
        return response

    def save_calibration(self, _, response: ehm.srv.SaveCalibration.Response):
        response.success = False
        if self.last_calibration:
            try:
                filepath = save_calibration(self.last_calibration)
                self.get_logger().info(f'Calibration saved to {filepath}')
                response.success = True
                response.filepath.data = str(filepath)
            except Exception as e:
                self.get_logger().error(f'Could not save calibration')
                self.get_logger().error(f'Underlying exception: {e}')
        return response

    # TODO: evaluation


def main(args=None):
    rclpy.init(args=args)

    handeye_server = HandeyeServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(handeye_server)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        handeye_server.destroy_node()


if __name__ == '__main__':
    main()
