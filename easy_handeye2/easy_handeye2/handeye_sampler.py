from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import rclpy
from rclpy.time import Duration, Time
from easy_handeye2.handeye_calibration import HandeyeCalibrationParameters

class HandeyeSampler:
    """
    Manages the samples acquired from tf.
    """

    def __init__(self, node: rclpy.node.Node, handeye_parameters: HandeyeCalibrationParameters):
        self.node = node
        self.handeye_parameters = handeye_parameters

        # tf structures
        self.tfBuffer = Buffer()
        """
        used to get transforms to build each sample

        :type: tf2_ros.Buffer
        """
        self.tfListener = TransformListener(self.tfBuffer, self.node, spin_thread=True)
        """
        used to get transforms to build each sample

        :type: tf2_ros.TransformListener
        """
        self.tfBroadcaster = TransformBroadcaster(self.node)
        """
        used to publish the calibration after saving it

        :type: tf.TransformBroadcaster
        """

        # internal input data
        self.samples = []
        """
        list of acquired samples

        Each sample is a dictionary going from 'rob' and 'opt' to the relative sampled transform in tf tuple format.

        :type: list[dict[str, ((float, float, float), (float, float, float, float))]]
        """

    def _wait_for_tf_init(self):
        """
        Waits until all needed frames are present in tf.

        :rtype: None
        """
        self.tfBuffer.lookup_transform(self.handeye_parameters.robot_base_frame,
                                       self.handeye_parameters.robot_effector_frame, Time(),
                                       Duration(seconds=20))
        self.tfBuffer.lookup_transform(self.handeye_parameters.tracking_base_frame,
                                       self.handeye_parameters.tracking_marker_frame, Time(),
                                       Duration(seconds=60))

    def _get_transforms(self, time=None):
        """
        Samples the transforms at the given time.

        :param time: sampling time (now if None)
        :type time: None|Time
        :rtype: dict[str, ((float, float, float), (float, float, float, float))]
        """
        if time is None:
            time = self.node.get_clock().now()

        # here we trick the library (it is actually made for eye_on_hand only). Trust me, I'm an engineer
        if self.handeye_parameters.eye_on_hand:
            rob = self.tfBuffer.lookup_transform(self.handeye_parameters.robot_base_frame,
                                                 self.handeye_parameters.robot_effector_frame, time,
                                                 Duration(seconds=10))
        else:
            rob = self.tfBuffer.lookup_transform(self.handeye_parameters.robot_effector_frame,
                                                 self.handeye_parameters.robot_base_frame, time,
                                                 Duration(seconds=10))
        opt = self.tfBuffer.lookup_transform(self.handeye_parameters.tracking_base_frame,
                                             self.handeye_parameters.tracking_marker_frame, time,
                                             Duration(seconds=10))
        return {'robot': rob, 'optical': opt}

    def take_sample(self):
        """
        Samples the transformations and appends the sample to the list.

        :rtype: None
        """
        self.node.get_logger().info("Taking a sample...")
        self.node.get_logger().info("all frames: "+self.tfBuffer.all_frames_as_string())
        transforms = self._get_transforms()
        self.node.get_logger().info("Got a sample")
        self.samples.append(transforms)

    def remove_sample(self, index):
        """
        Removes a sample from the list.

        :type index: int
        :rtype: None
        """
        if 0 <= index < len(self.samples):
            del self.samples[index]

    def get_samples(self):
        """
        Returns the samples accumulated so far.
        :rtype: [dict[str, ((float, float, float), (float, float, float, float))]]
        :return: A list of tuples containing the tracking and the robot transform pairs
        """
        return self.samples
