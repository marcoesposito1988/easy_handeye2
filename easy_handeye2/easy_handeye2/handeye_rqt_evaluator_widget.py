import pathlib

import numpy as np
from ament_index_python import get_resource
from easy_handeye2.handeye_calibration import load_calibration
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from rclpy.node import ParameterDescriptor, ParameterType
from rclpy.time import Duration, Time
from tf2_ros import TransformListener, Buffer, LookupException, ExtrapolationException, ConnectivityException

try:
    from python_qt_binding.QtGui import QWidget, QListWidgetItem, QLabel
except ImportError:
    try:
        from python_qt_binding.QtWidgets import QWidget, QListWidgetItem, QLabel, QVBoxLayout
    except:
        raise ImportError('Could not import QWidgets')


class RqtHandeyeEvaluatorWidget(QWidget):
    def __init__(self, parent, context):
        super(RqtHandeyeEvaluatorWidget, self).__init__()
        self._parent = parent
        self._plugin_context = context

        self._node = context.node
        # self.parameters_provider = HandeyeCalibrationParametersProvider(self._node)
        # self.parameters = self.parameters_provider.read()
        self._node.declare_parameter('name', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        name = self._node.get_parameter('name').get_parameter_value().string_value
        self.calibration = load_calibration(name)
        self.parameters = self.calibration.parameters

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        self._infoWidget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        _, package_path = get_resource('packages', 'easy_handeye2')
        ui_dir = pathlib.Path(package_path) / 'share' / 'easy_handeye2' / 'resource'

        ui_file = ui_dir / 'rqt_handeye_evaluator.ui'
        ui_info_file = ui_dir / 'rqt_handeye_info.ui'
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        loadUi(ui_info_file, self._infoWidget)
        self._widget.layout().insertWidget(0, self._infoWidget)
        # Give QObjects reasonable names
        self._widget.setObjectName('RqtHandeyeCalibrationUI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._infoWidget.calibNameLineEdit.setText(self.parameters.name)
        self._infoWidget.trackingBaseFrameLineEdit.setText(self.parameters.tracking_base_frame)
        self._infoWidget.trackingMarkerFrameLineEdit.setText(self.parameters.tracking_marker_frame)
        self._infoWidget.robotBaseFrameLineEdit.setText(self.parameters.robot_base_frame)
        self._infoWidget.robotEffectorFrameLineEdit.setText(self.parameters.robot_effector_frame)
        if self.parameters.calibration_type == 'eye_in_hand':
            self._infoWidget.calibTypeLineEdit.setText("eye in hand")
        else:
            self._infoWidget.calibTypeLineEdit.setText("eye on base")

        self.output_label = self._widget.label_message
        self.output_label.setText('Waiting for samples...')

        self._widget.pushButton_reset.clicked.connect(self.reset)

        self.is_eye_in_hand = self.parameters.calibration_type == 'eye_in_hand'
        self.robot_base_frame = self.parameters.robot_base_frame
        self.robot_effector_frame = self.parameters.robot_effector_frame

        self.tracking_measurement_frame = self.parameters.tracking_marker_frame
        if self.is_eye_in_hand:
            self.robot_measurement_frame = self.robot_base_frame
        else:
            self.robot_measurement_frame = self.robot_effector_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self._node, spin_thread=True)

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.tick)
        self.update_timer.start(500)

        self.last_robot_transform = None  # used to see if we are in a steady state

        self.measurement_transforms = []  # used to measure the quality of the calibration: should have the same value
        self.robot_transforms = []  # used to determine when to sample: we wait for a steady state that has not been sampled yet

        self._widget.show()

    def tick(self):
        # wait for steady state to avoid problems with lag
        # if next to a copy already in the dataset, tell to move
        # if could not sample transform, tell how old the last one is
        # show average error; show dataset in 3D?
        try:
            new_robot_transform = self.tf_buffer.lookup_transform(self.robot_base_frame, self.robot_effector_frame,
                                                                  Time(),
                                                                  Duration(seconds=0.2))
            new_measurement_transform = self.tf_buffer.lookup_transform(self.robot_measurement_frame,
                                                                        self.tracking_measurement_frame,
                                                                        Time(),
                                                                        Duration(seconds=0.2))
            if new_robot_transform is None:
                self._node.get_logger().warn(
                    'Could not sample transform between {} and {}'.format(self.robot_base_frame,
                                                                          self.robot_effector_frame))
            if new_measurement_transform is None:
                self._node.get_logger().warn(
                    'Could not sample transform between {} and {}'.format(self.robot_measurement_frame,
                                                                          self.tracking_measurement_frame))
            if self.last_robot_transform is None:
                self.last_robot_transform = new_robot_transform
                self.updateUI()
                self._node.get_logger().info('Sampled first transform')
                return
            if RqtHandeyeEvaluatorWidget.transform_too_far(new_robot_transform, self.last_robot_transform,
                                                           absolute_tolerance=0.001):
                self.last_robot_transform = new_robot_transform
                msg = 'Waiting for steady state'
                self._node.get_logger().info(msg)
                self.output_label.setText(msg)
                return

            if self.robot_transform_is_too_close_to_previous_sample(new_robot_transform, absolute_tolerance=0.003):
                self.updateUI()
                self._node.get_logger().info('Now we have {} samples\ntoo close to an old pose, move around!'.format(
                    len(self.measurement_transforms)))
                self.output_label.setText('Too close to an old pose, move around!')
                return

            self.robot_transforms.append(new_robot_transform)
            self.measurement_transforms.append(new_measurement_transform)
            self._node.get_logger().info('Appending transform; we got {} now'.format(len(self.measurement_transforms)))
            self.updateUI()
            # TODO: provide feedback if the data is sufficient

        except (LookupException, ExtrapolationException, ConnectivityException) as e:
            self.node.get_logger().error(
                'The specified tf frames for the robot base and hand do not seem to be connected')
            self.node.get_logger().error('Run the following command and check its output:')
            self.node.get_logger().error(
                f'ros2 run tf2_ros tf2_echo {self.robot_base_frame} {self.robot_effector_frame}')
            self.node.get_logger().error(
                f'You may need to correct the base_frame or effector_frame argument passed to the easy_handeye2 launch file')
            self.node.get_logger().error(f'Underlying tf exception: {e}')
            return False

    def reset(self):
        self.robot_transforms = []
        self.measurement_transforms = []
        self.updateUI()

    def updateUI(self):
        self._widget.spinBox_samples.setValue(len(self.measurement_transforms))
        if len(self.measurement_transforms) > 2:
            def translation_from_msg(msg):
                t = msg.transform.translation
                return t.x, t.y, t.z

            translations = [translation_from_msg(t) for t in self.measurement_transforms]
            translations_np = np.array(translations)
            translations_avg = translations_np.mean(axis=0)
            translations_from_avg = translations_np - translations_avg
            translations_max_divergence = np.max(translations_from_avg)
            self._node.get_logger().info("Maximum divergence: {}".format(translations_max_divergence))

            self._widget.doubleSpinBox_error.setEnabled(True)
            self._widget.doubleSpinBox_error.setValue(translations_max_divergence.max())
        else:
            self._widget.doubleSpinBox_error.setValue(0)
            self._widget.doubleSpinBox_error.setEnabled(False)

    @staticmethod
    def transform_to_concatenated_translation_quaternion(transform):
        tr = transform.transform.translation
        quat = transform.transform.rotation
        return np.array([tr.x, tr.y, tr.z, quat.x, quat.y, quat.z, quat.w])

    def robot_transform_is_too_close_to_previous_sample(self, new_robot_transform, absolute_tolerance):
        # TODO: use a meaningful metric
        posevec = RqtHandeyeEvaluatorWidget.transform_to_concatenated_translation_quaternion(new_robot_transform)
        for t in reversed(self.robot_transforms):
            old_posevec = RqtHandeyeEvaluatorWidget.transform_to_concatenated_translation_quaternion(t)
            if np.allclose(posevec, old_posevec, atol=absolute_tolerance):
                return True
        return False

    @staticmethod
    def transform_too_far(t1, t2, absolute_tolerance):
        # TODO: use a meaningful metric
        return not np.allclose(RqtHandeyeEvaluatorWidget.transform_to_concatenated_translation_quaternion(t1),
                               RqtHandeyeEvaluatorWidget.transform_to_concatenated_translation_quaternion(t2),
                               atol=absolute_tolerance)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

        # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
