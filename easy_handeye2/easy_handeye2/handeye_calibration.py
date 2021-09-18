import os
import yaml
from easy_handeye2_msgs.msg import HandeyeCalibration
from rosidl_runtime_py import set_message_fields, message_to_yaml

from . import CALIBRATIONS_DIRECTORY


def filepath_for_calibration(name):
    return CALIBRATIONS_DIRECTORY / f'{name}.calib'


def load_calibration(name):
    filepath = filepath_for_calibration(name)
    with open(filepath) as f:
        m = yaml.load(f.read())
    ret = HandeyeCalibration()
    set_message_fields(ret, m)
    return ret


def save_calibration(calibration: HandeyeCalibration) -> bool:
    if not os.path.exists(CALIBRATIONS_DIRECTORY):
        os.makedirs(CALIBRATIONS_DIRECTORY)
    filepath = filepath_for_calibration(calibration.parameters.name)
    with open(filepath, 'w') as f:
        f.write(message_to_yaml(calibration))
    return True
