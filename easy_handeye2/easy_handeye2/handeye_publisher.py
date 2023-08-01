#!/usr/bin/env python

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import ParameterType, ParameterDescriptor
import tf2_ros
import geometry_msgs.msg
from easy_handeye2.handeye_calibration import load_calibration


class HandeyePublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__('handeye_publisher')

        self.declare_parameter('name', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        name = self.get_parameter('name').get_parameter_value().string_value

        self.get_logger().info(f'Loading the calibration with name {name}')

        self.calibration = load_calibration(name)
        parameters = self.calibration.parameters

        if parameters.calibration_type == 'eye_in_hand':
            orig = parameters.robot_effector_frame
        else:
            orig = parameters.robot_base_frame
        dest = parameters.tracking_base_frame

        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.static_transformStamped = geometry_msgs.msg.TransformStamped()

        self.static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        self.static_transformStamped.header.frame_id = orig
        self.static_transformStamped.child_frame_id = dest

        self.static_transformStamped.transform = self.calibration.transform

        self.broadcaster.sendTransform(self.static_transformStamped)


def main(args=None):
    rclpy.init(args=args)

    handeye_publisher = HandeyePublisher()

    try:
        rclpy.spin(handeye_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        handeye_publisher.destroy_node()


if __name__ == '__main__':
    main()
