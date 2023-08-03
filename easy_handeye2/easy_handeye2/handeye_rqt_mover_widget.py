import sys

from python_qt_binding.QtCore import QCoreApplication, Qt, Signal
from python_qt_binding.QtWidgets import QWidget, QApplication, QVBoxLayout, QHBoxLayout, QProgressBar, QLabel, \
    QPushButton

from easy_handeye2.handeye_calibration import HandeyeCalibrationParametersProvider
from easy_handeye2.handeye_client import HandeyeClient


class RqtHandeyeMoverWidget(QWidget):
    _TITLE_PLUGIN = 'HandEye Mover'

    # To be connected to PluginContainerWidget
    sig_sysmsg = Signal(str)
    sig_sysprogress = Signal(str)

    NOT_INITED_YET = 0
    BAD_PLAN = 1
    GOOD_PLAN = 2
    MOVED_TO_POSE = 3
    BAD_STARTING_POSITION = 4
    GOOD_STARTING_POSITION = 5
    CHECKING_STARTING_POSITION = 6
    MOVEMENT_FAILED = 7

    def __init__(self, context, node=None):
        super(RqtHandeyeMoverWidget, self).__init__()
        self.setObjectName(self._TITLE_PLUGIN)
        self.setWindowTitle(self._TITLE_PLUGIN)

        self._plugin_context = context
        self._node = context.node

        self.parameters_provider = HandeyeCalibrationParametersProvider(self._node)
        self.parameters = self.parameters_provider.read()

        self.handeye_client = HandeyeClient(self._node, self.parameters)

        self.current_target_pose = -1  # -1 is home
        self.target_poses = None
        self.plan_was_successful = None
        self.state = RqtHandeyeMoverWidget.NOT_INITED_YET

        self.layout = QVBoxLayout()
        self.labels_layout = QHBoxLayout()
        self.buttons_layout = QHBoxLayout()

        self.progress_bar = QProgressBar()
        self.pose_number_lbl = QLabel('0/0')
        self.bad_plan_lbl = QLabel('No plan yet')
        self.bad_plan_lbl.setAlignment(Qt.AlignCenter)
        self.guide_lbl = QLabel('Hello')
        self.guide_lbl.setWordWrap(True)

        self.check_start_pose_btn = QPushButton('Check starting pose')
        self.check_start_pose_btn.clicked.connect(self.handle_check_current_state)

        self.next_pose_btn = QPushButton('Next Pose')
        self.next_pose_btn.clicked.connect(self.handle_next_pose)

        self.plan_btn = QPushButton('Plan')
        self.plan_btn.clicked.connect(self.handle_plan)

        self.execute_btn = QPushButton('Execute')
        self.execute_btn.clicked.connect(self.handle_execute)

        self.labels_layout.addWidget(self.pose_number_lbl)
        self.labels_layout.addWidget(self.bad_plan_lbl)

        self.buttons_layout.addWidget(self.check_start_pose_btn)
        self.buttons_layout.addWidget(self.next_pose_btn)
        self.buttons_layout.addWidget(self.plan_btn)
        self.buttons_layout.addWidget(self.execute_btn)

        self.layout.addWidget(self.progress_bar)
        self.layout.addLayout(self.labels_layout)
        self.layout.addWidget(self.guide_lbl)
        self.layout.addLayout(self.buttons_layout)

        self.setLayout(self.layout)

        self.plan_btn.setEnabled(False)
        self.execute_btn.setEnabled(False)

        self.setWindowTitle('Easy HandEye Mover')
        self.show()

    def update_ui(self):
        if self.target_poses:
            count_target_poses = len(self.target_poses)
        else:
            count_target_poses = 1
        self.progress_bar.setMaximum(count_target_poses)
        self.progress_bar.setValue(self.current_target_pose + 1)
        self.pose_number_lbl.setText('{}/{}'.format(self.current_target_pose + 1, count_target_poses))

        if self.state == RqtHandeyeMoverWidget.BAD_PLAN:
            self.bad_plan_lbl.setText('BAD plan!! Don\'t do it!!!!')
            self.bad_plan_lbl.setStyleSheet('QLabel { background-color : red}')
        elif self.state == RqtHandeyeMoverWidget.GOOD_PLAN:
            self.bad_plan_lbl.setText('Good plan')
            self.bad_plan_lbl.setStyleSheet('QLabel { background-color : green}')
        else:
            self.bad_plan_lbl.setText('No plan yet')
            self.bad_plan_lbl.setStyleSheet('')

        if self.state == RqtHandeyeMoverWidget.NOT_INITED_YET:
            self.guide_lbl.setText(
                'Bring the robot to a plausible position and check if it is a suitable starting pose')
        elif self.state == RqtHandeyeMoverWidget.CHECKING_STARTING_POSITION:
            self.guide_lbl.setText(
                'Checking if the robot can translate and rotate in all directions from the current pose')
        elif self.state == RqtHandeyeMoverWidget.BAD_STARTING_POSITION:
            self.guide_lbl.setText('Cannot calibrate from current position')
        elif self.state == RqtHandeyeMoverWidget.GOOD_STARTING_POSITION:
            self.guide_lbl.setText('Ready to start: click on next pose')
        elif self.state == RqtHandeyeMoverWidget.GOOD_PLAN:
            self.guide_lbl.setText('The plan seems good: press execute to move the robot')
        elif self.state == RqtHandeyeMoverWidget.BAD_PLAN:
            self.guide_lbl.setText('Planning failed: try again')
        elif self.state == RqtHandeyeMoverWidget.MOVED_TO_POSE:
            self.guide_lbl.setText('Pose reached: take a sample and go on to next pose')

        can_plan = self.state == RqtHandeyeMoverWidget.GOOD_STARTING_POSITION
        self.plan_btn.setEnabled(can_plan)
        can_move = self.state == RqtHandeyeMoverWidget.GOOD_PLAN
        self.execute_btn.setEnabled(can_move)
        QCoreApplication.processEvents()

    def handle_check_current_state(self):
        self.state = RqtHandeyeMoverWidget.CHECKING_STARTING_POSITION
        self.update_ui()
        res = self.handeye_client.check_starting_pose()
        if res.can_calibrate:
            self.state = RqtHandeyeMoverWidget.GOOD_STARTING_POSITION
        else:
            self.state = RqtHandeyeMoverWidget.BAD_STARTING_POSITION
        self.current_target_pose = res.target_poses.current_target_pose_index
        self.target_poses = res.target_poses.target_poses
        self.plan_was_successful = None

        self.update_ui()

    def handle_next_pose(self):
        res = self.handeye_client.select_target_pose(self.current_target_pose+1)
        self.current_target_pose = res.target_poses.current_target_pose_index
        self.target_poses = res.target_poses.target_poses
        self.plan_was_successful = None

        self.state = RqtHandeyeMoverWidget.GOOD_STARTING_POSITION
        self.update_ui()

    def handle_plan(self):
        self.guide_lbl.setText('Planning to the next position. Click on execute when a good one was found')
        res = self.handeye_client.plan_to_selected_target_pose()
        self.plan_was_successful = res.success
        if self.plan_was_successful:
            self.state = RqtHandeyeMoverWidget.GOOD_PLAN
        else:
            self.state = RqtHandeyeMoverWidget.BAD_PLAN
        self.update_ui()

    def handle_execute(self):
        if self.plan_was_successful:
            self.guide_lbl.setText('Going to the selected pose')
            res = self.handeye_client.execute_plan()
            if res.success:
                self.state = RqtHandeyeMoverWidget.MOVED_TO_POSE
            else:
                self.state = RqtHandeyeMoverWidget.MOVEMENT_FAILED
            self.update_ui()

    def emit_sysmsg(self, msg_str):
        self.sig_sysmsg.emit(msg_str)


if __name__ == '__main__':
    # main should be used only for debug purpose.
    # This launches this QWidget as a standalone rqt gui.
    from rqt_gui.main import Main

    main = Main()
    sys.exit(main.main(sys.argv, standalone='rqt_handeye_mover'))
