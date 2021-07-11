#!/usr/bin/env python

import rclpy
import tf2_ros
import geometry_msgs.msg
from easy_handeye2.handeye_calibration import HandeyeCalibration


class HandeyePublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__('handeye_publisher')

        self.declare_parameter('inverse', False)
        self.declare_parameter('calibration_file')

        inverse = self.get_parameter('inverse').get_parameter_value().bool_value  # TODO implement or remove
        filename = self.get_parameter('calibration_file').get_parameter_value().string_value

        if filename == '':
            self.get_logger().debug('No path specified for the calibration file, loading from the standard location')
            filename = HandeyeCalibration.filename_for_namespace(self.get_namespace())

        self.get_logger().info("Loading the calibration from file: %s", filename)
        calib = HandeyeCalibration.from_filename(filename)

        if calib.parameters.eye_on_hand:
            self.declare_parameter('robot_effector_frame')
            overriding_robot_effector_frame = self.get_parameter('robot_effector_frame').get_parameter_value().string_value
            if overriding_robot_effector_frame != "":
                calib.transformation.header.frame_id = overriding_robot_effector_frame
        else:
            self.declare_parameter('robot_base_frame')
            overriding_robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
            if overriding_robot_base_frame != "":
                calib.transformation.header.frame_id = overriding_robot_base_frame
        self.declare_parameter('tracking_base_frame')
        overriding_tracking_base_frame = self.get_parameter('tracking_base_frame').get_parameter_value().string_value
        if overriding_tracking_base_frame != "":
            calib.transformation.child_frame_id = overriding_tracking_base_frame

        self.get_logger().info('loading calibration parameters into namespace {}'.format(
            self.get_logger().get_namespace()))
        HandeyeCalibration.store_to_parameter_server(self, calib)  # TODO implement

        orig = calib.transformation.header.frame_id  # tool or base link
        dest = calib.transformation.child_frame_id  # tracking_base_frame

        broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = self.get_clock().now()
        static_transformStamped.header.frame_id = orig
        static_transformStamped.child_frame_id = dest

        static_transformStamped.transform = calib.transformation.transform

        broadcaster.sendTransform(static_transformStamped)


def main(args=None):
    rclpy.init(args=args)

    handeye_publisher = HandeyePublisher()

    rclpy.spin(handeye_publisher)

    handeye_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
