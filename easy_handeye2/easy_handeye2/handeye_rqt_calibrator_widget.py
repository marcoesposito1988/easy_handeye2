
import pathlib

import rclpy
from ament_index_python import get_resource
from easy_handeye2.handeye_calibration import HandeyeCalibrationParametersProvider
from python_qt_binding import loadUi
from easy_handeye2.handeye_client import HandeyeClient

from python_qt_binding.QtWidgets import QWidget, QListWidgetItem, QLabel, QComboBox, QHBoxLayout


def format_sample(sample):
    x, y, z = sample.translation.x, sample.translation.y, sample.translation.z
    qx, qy, qz, qw = sample.rotation.x, sample.rotation.y, sample.rotation.z, sample.rotation.w
    return 'translation: [{:+.2f}, {:+.2f}, {:+.2f}]\nrotation: [{:+.2f}, {:+.2f}, {:+.2f}, {:+.2f}]'.format(x, y, z, qx, qy, qz, qw)


class RqtHandeyeCalibratorWidget(QWidget):
    def __init__(self, parent, context):
        super(RqtHandeyeCalibratorWidget, self).__init__()
        self._parent = parent
        self._plugin_context = context

        self._node = context.node
        self.parameters_provider = HandeyeCalibrationParametersProvider(self._node)
        self.parameters = self.parameters_provider.read()

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

        self._calibration_algorithm_combobox = QComboBox()
        self._calibration_algorithm_layout = QHBoxLayout()
        self._calibration_algorithm_layout.insertWidget(0, QLabel('Calibration algorithm: '))
        self._calibration_algorithm_layout.insertWidget(1, self._calibration_algorithm_combobox)
        self._infoWidget.layout().insertLayout(-1, self._calibration_algorithm_layout)

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
            self._calibration_algorithm_combobox.insertItem(i, a)
        index_of_curr_alg = resp.algorithms.index(resp.current_algorithm)
        self._calibration_algorithm_combobox.setCurrentIndex(index_of_curr_alg)
        self._calibration_algorithm_combobox.currentTextChanged.connect(self.client.set_algorithm)

        self._infoWidget.calibNameLineEdit.setText(self.parameters.name)
        self._infoWidget.trackingBaseFrameLineEdit.setText(self.parameters.tracking_base_frame)
        self._infoWidget.trackingMarkerFrameLineEdit.setText(self.parameters.tracking_marker_frame)
        self._infoWidget.robotBaseFrameLineEdit.setText(self.parameters.robot_base_frame)
        self._infoWidget.robotEffectorFrameLineEdit.setText(self.parameters.robot_effector_frame)
        if self.parameters.eye_in_hand:
            self._infoWidget.calibTypeLineEdit.setText("eye in hand")
        else:
            self._infoWidget.calibTypeLineEdit.setText("eye on base")

        self._widget.takeButton.clicked[bool].connect(self.handle_take_sample)
        self._widget.removeButton.clicked[bool].connect(self.handle_remove_sample)
        self._widget.computeButton.clicked[bool].connect(self.handle_compute_calibration)
        self._widget.saveButton.clicked[bool].connect(self.handle_save_calibration)

        self._widget.removeButton.setEnabled(False)
        self._widget.computeButton.setEnabled(False)
        self._widget.saveButton.setEnabled(False)

        sample_list = self.client.get_sample_list()
        self._display_sample_list(sample_list)
        self._widget.computeButton.setEnabled(len(sample_list.samples) > 1)

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

    def handle_take_sample(self):
        sample_list = self.client.take_sample()
        self._display_sample_list(sample_list)
        self._widget.computeButton.setEnabled(len(sample_list.samples) > 1)
        self._widget.saveButton.setEnabled(False)

    def handle_remove_sample(self):
        index = self._widget.sampleListWidget.currentRow()
        sample_list = self.client.remove_sample(index)
        self._display_sample_list(sample_list)
        self._widget.computeButton.setEnabled(len(sample_list.samples) > 1)
        self._widget.saveButton.setEnabled(False)

    def handle_compute_calibration(self):
        result = self.client.compute_calibration()
        self._widget.computeButton.setEnabled(False)
        if result.valid:
            tr = result.calibration.transform.translation
            qt = result.calibration.transform.rotation
            t = f'Translation\n\tx: {tr.x:.6f}\n\ty: {tr.y:.6f}\n\tz: {tr.z:.6f})\nRotation\n\tx: {qt.x:.6f}\n\ty: {qt.y:.6f}\n\tz: {qt.z:.6f}\n\tw: {qt.w:.6f}'
            self._widget.outputBox.setPlainText(t)
            self._widget.saveButton.setEnabled(True)
        else:
            self._widget.outputBox.setPlainText('The calibration could not be computed')
            self._widget.saveButton.setEnabled(False)

    def handle_save_calibration(self):
        self.client.save()
        self._widget.saveButton.setEnabled(False)

