from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from wpimath import units
from math import pi
from typing import Callable

from subsystems.drive import Drive
from subsystems.vision import Vision
from commands.target_tag import TargetTag

import config


class TargetHPS(TargetTag):
    def __init__(self, drive: Drive, vision: Vision, left_hps: bool):
        self.drive_ = drive
        self.vision_ = vision

        self.target_angle_ = units.degreesToRadians(48)
        if left_hps:
            self.target_angle_ *= -1
        if config.is_red():
            self.target_angle_ += pi

        match (left_hps, config.is_red()):
            case (False, False):
                self.tag_id_ = 12
            case (False, True):
                self.tag_id_ = 2
            case (True, False):
                self.tag_id_ = 13
            case (True, True):
                self.tag_id_ = 1

        self.left_target_ = units.degreesToRadians(9.4)
        self.right_target_ = units.degreesToRadians(12.234)

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
        return 1

    @property
    def right_i(self) -> int:
        return self.left_i

    @property
    def get_left(self) -> Callable[[PhotonTrackedTarget], float]:
        return lambda target: -target.getYaw()

    @property
    def get_right(self) -> Callable[[PhotonTrackedTarget], float]:
        return lambda target: target.getPitch()

    @property
    def use_diag(self) -> bool:
        return False
