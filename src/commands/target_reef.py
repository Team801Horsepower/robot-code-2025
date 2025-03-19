from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from wpimath import units
from math import pi
from typing import Callable, Tuple
from wpilib import SmartDashboard

from subsystems.drive import Drive
from subsystems.vision import Vision
from commands.target_tag import TargetTag

import config


class TargetReef(TargetTag):
    def __init__(self, drive: Drive, vision: Vision, stalk_i: int, algae: bool):
        self.drive_ = drive
        self.vision_ = vision

        tag_ids = [10, 11, 6, 7, 8, 9] if config.is_red() else [21, 20, 19, 18, 17, 22]
        self.face_i = int(stalk_i / 2)
        self.tag_id_ = tag_ids[self.face_i]
        self.target_angle_ = 0.0 if config.is_red() else pi
        self.target_angle_ += pi / 3 * self.face_i

        if not algae:
            left_stalk = stalk_i % 2 == 0
            # self.left_target_ = units.degreesToRadians(13.6 if left_stalk else -12.11)
            # self.right_target_ = units.degreesToRadians(13.75 if left_stalk else -11.66)
            self.left_target_ = units.degreesToRadians(11.43 if left_stalk else -12.05)
            self.right_target_ = units.degreesToRadians(16.97 if left_stalk else -7.19)
        else:
            self.right_target_ = units.degreesToRadians(3.815)
            self.left_target_ = units.degreesToRadians(-3.815)

        super().__init__()

    @property
    def drive(self) -> Drive:
        return self.drive_

    @property
    def vision(self) -> Vision:
        return self.vision_

    @property
    def tag_id(self) -> int:
        return self.tag_id_

    @property
    def approach_pid_constants(self) -> Tuple[float, float, float]:
        p = SmartDashboard.getNumber("reef approach P", 10.0)
        i = SmartDashboard.getNumber("reef approach I", 0.0)
        return (p, 0.0, i)

    @property
    def strafe_pid_constants(self) -> Tuple[float, float, float]:
        p = SmartDashboard.getNumber("reef strafe P", 3.0)
        i = SmartDashboard.getNumber("reef strafe I", 0.0)
        return (p, 0.0, i)

    @property
    def theta_pid_constants(self) -> Tuple[float, float, float]:
        return (6.5, 0.0, 0.0)

    @property
    def target_angle(self) -> float:
        return self.target_angle_

    @property
    def left_target(self) -> float:
        return self.left_target_

    @property
    def right_target(self) -> float:
        return self.right_target_

    @property
    def left_i(self) -> int:
        return 0

    @property
    def right_i(self) -> int:
        return 2

    @property
    def get_left(self) -> Callable[[PhotonTrackedTarget], float]:
        return lambda target: target.getYaw()

    @property
    def get_right(self) -> Callable[[PhotonTrackedTarget], float]:
        return self.get_left

    @property
    def align_speed(self) -> float:
        # return 4.5
        return SmartDashboard.getNumber("reef align speed", 5.5)

    @property
    def use_diag(self) -> bool:
        return False
