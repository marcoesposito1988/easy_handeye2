#!/usr/bin/env python

import rclpy
from easy_handeye2.handeye_client import HandeyeClient
from rclpy.executors import ExternalShutdownException


# for reading single character without hitting RETURN (unless it's ipython!)
def getchar():
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class HandeyeCalibrationCommander(rclpy.node.Node):
    def __init__(self):
        super().__init__('handeye_calibration_commander')

        print('Handeye Calibration Commander')
        print('connecting to: {}'.format((self.get_namespace().strip('/'))))

        self.client = HandeyeClient(self)

        if self.client.parameters.calibration_type == 'eye_in_hand':
            print('eye-on-hand calibration')
            print('robot effector frame: {}'.format(self.client.parameters.robot_effector_frame))
        else:
            print('eye-on-base calibration')
            print('robot base frame: {}'.format(self.client.parameters.robot_base_frame))
        print('tracking base frame: {}'.format(self.client.parameters.tracking_base_frame))
        print('tracking target frame: {}'.format(self.client.parameters.tracking_marker_frame))

    def _take_menu(self):
        print('Press SPACE to take a sample or ENTER to continue\n')
        i = getchar()
        if i == ' ':
            self.client.take_sample()

    def _display_sample_list(self, sample_list):
        for i in range(len(sample_list.hand_world_samples)):
            print('{}) \n hand->world {} \n camera->marker {}\n'.format(i,
                                       sample_list.hand_world_samples[i],
                                       sample_list.camera_marker_samples[i]))

    def edit_menu(self):
        while len(self.client.get_sample_list().hand_world_samples) > 0:
            prompt_str = 'Press a number and ENTER to delete the respective sample, or ENTER to continue:\n'
            self._display_sample_list(self.client.get_sample_list())
            sample_to_delete = input(prompt_str)
            if sample_to_delete.isdigit():
                self.client.remove_sample(int(sample_to_delete))
            else:
                break

    def _save_menu(self):
        print('Press c to compute the calibration or ENTER to continue\n')
        i = getchar()
        if i == 'c':
            cal = self.client.compute_calibration()
            print(cal)
        print('Press s to save the calibration to parameters and namespace, q to quit or ENTER to continue\n')
        i = getchar()
        if i == 's':
            self.client.save()
        elif i == 'q':
            quit()

    def interactive_menu(self):
        self._take_menu()
        self.edit_menu()
        self._save_menu()


def main(args=None):
    rclpy.init(args=args)

    handeye_calibration_commander = HandeyeCalibrationCommander()

    try:
        handeye_calibration_commander.edit_menu()  # the sample list might not be empty when we start the commander
        while True:
            rclpy.spin(handeye_calibration_commander)
            handeye_calibration_commander.interactive_menu()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        handeye_calibration_commander.destroy_node()


if __name__ == '__main__':
    main()