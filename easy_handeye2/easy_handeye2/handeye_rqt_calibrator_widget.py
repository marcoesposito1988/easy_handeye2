import math
import pathlib

import numpy as np
import transforms3d as tfs
from ament_index_python import get_resource
from easy_handeye2.handeye_calibration import HandeyeCalibrationParametersProvider
from easy_handeye2.handeye_client import HandeyeClient
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QWidget


def format_sample(sample):
    x, y, z = sample.translation.x, sample.translation.y, sample.translation.z
    qx, qy, qz, qw = sample.rotation.x, sample.rotation.y, sample.rotation.z, sample.rotation.w
    return 'translation: [{:+.2f}, {:+.2f}, {:+.2f}]\nrotation: [{:+.2f}, {:+.2f}, {:+.2f}, {:+.2f}]'.format(x, y, z,
                                                                                                             qx, qy, qz,
                                                                                                             qw)


class RqtHandeyeCalibratorWidget(QWidget):
    def __init__(self, parent, context):
        super(RqtHandeyeCalibratorWidget, self).__init__()
        self._parent = parent
        self._plugin_context = context

        self._node = context.node
        self.parameters_provider = HandeyeCalibrationParametersProvider(self._node)
        self.parameters = self.parameters_provider.read()

        self._current_transforms = None

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

        # Create QWidgets
        self._widget = QWidget()
        self._infoWidget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        _, package_path = get_resource('packages', 'easy_handeye2')
        ui_dir = pathlib.Path(package_path) / 'share' / 'easy_handeye2' / 'resource'
        ui_file = ui_dir / 'rqt_handeye.ui'
        ui_info_file = ui_dir / 'rqt_handeye_info.ui'
        # Extend the widget with all attributes and children from UI file
        loadUi(str(ui_file.resolve()), self._widget)
        loadUi(str(ui_info_file.resolve()), self._infoWidget)
        self._widget.horizontalLayout_infoAndActions.insertWidget(0, self._infoWidget)

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

        self.client = HandeyeClient(self._node, self.parameters)

        resp = self.client.list_algorithms()
        for i, a in enumerate(resp.algorithms):
            self._widget.calibAlgorithmComboBox.insertItem(i, a)
        index_of_curr_alg = resp.algorithms.index(resp.current_algorithm)
        self._widget.calibAlgorithmComboBox.setCurrentIndex(index_of_curr_alg)
        self._widget.calibAlgorithmComboBox.currentTextChanged.connect(self.client.set_algorithm)

        self._infoWidget.calibNameLineEdit.setText(self.parameters.name)
        self._infoWidget.trackingBaseFrameLineEdit.setText(self.parameters.tracking_base_frame)
        self._infoWidget.trackingMarkerFrameLineEdit.setText(self.parameters.tracking_marker_frame)
        self._infoWidget.robotBaseFrameLineEdit.setText(self.parameters.robot_base_frame)
        self._infoWidget.robotEffectorFrameLineEdit.setText(self.parameters.robot_effector_frame)
        if self.parameters.calibration_type == 'eye_in_hand':
            self._infoWidget.calibTypeLineEdit.setText("eye in hand")
        else:
            self._infoWidget.calibTypeLineEdit.setText("eye on base")

        self._widget.takeButton.clicked[bool].connect(self.handle_take_sample)
        self._widget.removeButton.clicked[bool].connect(self.handle_remove_sample)
        self._widget.saveButton.clicked[bool].connect(self.handle_save_calibration)
        self._widget.calibAlgorithmComboBox.currentIndexChanged.connect(self.handle_compute_calibration)

        self._widget.removeButton.setEnabled(False)
        self._widget.saveButton.setEnabled(False)

        sample_list = self.client.get_sample_list()
        self._display_sample_list(sample_list)

        self._update_ui_timer = QTimer(self)
        self._update_ui_timer.timeout.connect(self._updateUI)
        self._update_ui_timer.start(100)

    def shutdown(self):
        self._update_ui_timer.stop()

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

    def _display_sample_list(self, sample_list):
        self._widget.sampleListWidget.clear()

        for i, s in enumerate(sample_list.samples):
            formatted_robot_sample = format_sample(s.robot)
            formatted_tracking_sample = format_sample(s.tracking)
            self._widget.sampleListWidget.addItem(
                '{}) \n hand->world \n {} \n camera->marker\n {}\n'.format(i + 1, formatted_robot_sample,
                                                                           formatted_tracking_sample))
        self._widget.sampleListWidget.setCurrentRow(len(sample_list.samples) - 1)
        self._widget.removeButton.setEnabled(len(sample_list.samples) > 0)

    @staticmethod
    def _translation_distance(t1, t2):
        cmt1 = t1.translation
        tr1 = np.array((cmt1.x, cmt1.y, cmt1.z))
        cmt2 = t2.translation
        tr2 = np.array((cmt2.x, cmt2.y, cmt2.z))
        return np.linalg.norm(tr1 - tr2)

    @staticmethod
    def _q_log(q):
        # transform to a unit quaternion
        u_q = np.array(q) / tfs.quaternions.qnorm(q)
        unit_q = tfs.quaternions.fillpositive(np.array(u_q[1:4]))
        log_result = np.zeros(3)
        if not np.allclose(unit_q[1:4], np.zeros(3)):
            log_result = np.arccos(unit_q[0]) * unit_q[1:4] / np.linalg.norm(unit_q[1:4])

        return log_result

    @staticmethod
    def _q_distance(q1, q2):
        u_q1 = np.array(q1) / tfs.quaternions.qnorm(q1)
        u_q2 = np.array(q2) / tfs.quaternions.qnorm(q2)
        q_1 = tfs.quaternions.fillpositive(np.array(u_q1[1:4]))
        q_2 = tfs.quaternions.fillpositive(np.array(u_q2[1:4]))
        delta_q = tfs.quaternions.qmult(q_1, tfs.quaternions.qconjugate(q_2))

        log_q = RqtHandeyeCalibratorWidget._q_log(delta_q)

        if not np.allclose(delta_q, np.array([-1, 0, 0, 0])):
            d = 2 * np.linalg.norm(log_q)
        else:
            d = 2 * np.pi

        return d

    @staticmethod
    def _rotation_distance(t1, t2):
        cmq1 = t1.rotation
        rot1 = (cmq1.w, cmq1.x, cmq1.y, cmq1.z)
        cmq2 = t2.rotation
        rot2 = (cmq2.w, cmq2.x, cmq2.y, cmq2.z)
        return RqtHandeyeCalibratorWidget._q_distance(rot1, rot2)

    @staticmethod
    def _has_moved(t1, t2):
        TRANSLATION_TOLERANCE_M = 0.003
        ROTATION_TOLERANCE_RAD = math.radians(3)

        translation_has_moved = RqtHandeyeCalibratorWidget._translation_distance(t1, t2) > TRANSLATION_TOLERANCE_M
        rotation_has_moved = RqtHandeyeCalibratorWidget._rotation_distance(t1, t2) > ROTATION_TOLERANCE_RAD
        return translation_has_moved or rotation_has_moved

    def _check_still_moving(self, new_transforms):
        if self._current_transforms is None:
            self._current_transforms = new_transforms
            return False

        robot_is_moving = RqtHandeyeCalibratorWidget._has_moved(new_transforms.robot, self._current_transforms.robot)
        tracking_is_moving = RqtHandeyeCalibratorWidget._has_moved(new_transforms.tracking,
                                                                   self._current_transforms.tracking)

        self._current_transforms = new_transforms

        return robot_is_moving or tracking_is_moving

    def _updateUI(self):
        new_transforms = self.client.get_current_transforms()
        if new_transforms is None or self._check_still_moving(new_transforms):
            self._widget.takeButton.setEnabled(False)
        else:
            self._widget.takeButton.setEnabled(True)

    def handle_take_sample(self):
        sample_list = self.client.take_sample()
        self._display_sample_list(sample_list)
        self._widget.saveButton.setEnabled(False)
        self.handle_compute_calibration()

    def handle_remove_sample(self):
        index = self._widget.sampleListWidget.currentRow()
        sample_list = self.client.remove_sample(index)
        self._display_sample_list(sample_list)
        self._widget.saveButton.setEnabled(False)

    def handle_compute_calibration(self):
        if len(self.client.get_sample_list().samples) > 2:
            result = self.client.compute_calibration()
            if result.valid:
                tr = result.calibration.transform.translation
                qt = result.calibration.transform.rotation
                t = f'Translation\n\tx: {tr.x:.6f}\n\ty: {tr.y:.6f}\n\tz: {tr.z:.6f})\nRotation\n\tx: {qt.x:.6f}\n\ty: {qt.y:.6f}\n\tz: {qt.z:.6f}\n\tw: {qt.w:.6f}'
                self._widget.outputBox.setPlainText(t)
                self._widget.saveButton.setEnabled(True)
            else:
                self._widget.outputBox.setPlainText('The calibration could not be computed')
                self._widget.saveButton.setEnabled(False)
        else:
            self._widget.outputBox.setPlainText('Too few samples, the calibration cannot not be computed')
            self._widget.saveButton.setEnabled(False)

    def handle_save_calibration(self):
        self.client.save()
        self._widget.saveButton.setEnabled(False)
