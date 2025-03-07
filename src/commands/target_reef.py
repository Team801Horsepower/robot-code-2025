from commands2 import Command
from wpimath.geometry import Pose2d, Transform2d, Rotation2d, Translation2d
from wpimath.controller import PIDController
from wpimath import units
from wpilib import SmartDashboard
from math import pi, sqrt
from typing import Optional
import time

from subsystems.drive import Drive
from subsystems.vision import Vision

import config


class TargetReef(Command):
    def __init__(self, drive: Drive, vision: Vision, stalk_i: int):
        self.drive = drive
        self.vision = vision

        tag_ids = [10, 11, 6, 7, 8, 9] if config.is_red() else [21, 20, 19, 18, 17, 22]
        self.tag_id = tag_ids[int(stalk_i / 2)]
        self.target_angle = 0 if config.is_red else pi
        self.target_angle += pi / 3 * int(stalk_i / 2)
        self.x_pid = PIDController(15.0, 0, 0)
        self.y_pid = PIDController(3.0, 0, 0)
        self.theta_pid = PIDController(5.0, 5.0, 0.3)

        # TODO: after front cameras are pitched down, remeasure these values
        a = units.degreesToRadians(12.3)
        b = units.degreesToRadians(11.8)
        # a = units.degreesToRadians(0)
        # b = units.degreesToRadians(13.9)
        left_stalk = stalk_i % 2 == 0
        self.left_target = a if left_stalk else -b
        self.right_target = b if left_stalk else -a

    def execute(self):
        new_p = SmartDashboard.getNumber("approach P", 15.0)
        self.x_pid.setP(new_p)

        rot = self.drive.odometry.rotation().radians()
        target_rot = self.target_angle
        while target_rot > rot + pi:
            target_rot -= 2 * pi
        while target_rot < rot - pi:
            target_rot += 2 * pi
        omega = self.theta_pid.calculate(rot, target_rot)

        def get_yaw(i: int) -> Optional[float]:
            targets = self.vision.results[i].getTargets()
            for target in targets:
                if target.getFiducialId() == self.tag_id:
                    return units.degreesToRadians(target.getYaw())
            return None

        left_yaw = get_yaw(0)
        right_yaw = get_yaw(2)

        if left_yaw is None or right_yaw is None:
            drive_input = Transform2d(0, 0, omega)
        else:
            # left_err = self.left_target - left_yaw
            # right_err = self.right_target - right_yaw

            SmartDashboard.putNumber("left yaw", left_yaw)
            SmartDashboard.putNumber("right yaw", right_yaw)
            SmartDashboard.putNumber("left target", self.left_target)
            SmartDashboard.putNumber("right target", self.right_target)

            left_power = self.y_pid.calculate(left_yaw, self.left_target)
            right_power = self.y_pid.calculate(right_yaw, self.right_target)
            strafe_speed = (left_power + right_power) / 2
            if abs(strafe_speed) > config.auto_drive_speed:
                strafe_speed = (
                    strafe_speed / abs(strafe_speed) * config.auto_drive_speed
                )
            SmartDashboard.putNumber("strafe power", strafe_speed)

            diff = left_yaw - right_yaw
            target_diff = self.left_target - self.right_target
            approach_speed = self.x_pid.calculate(diff, target_diff)
            if abs(approach_speed) > config.auto_drive_speed:
                approach_speed = (
                    approach_speed / abs(approach_speed) * config.auto_drive_speed
                )
            SmartDashboard.putNumber("diff", units.radiansToDegrees(diff))
            SmartDashboard.putNumber("diff target", units.radiansToDegrees(target_diff))
            SmartDashboard.putNumber("approach power", approach_speed)

            drive_input = Transform2d(approach_speed, strafe_speed, omega)

        self.drive.drive(drive_input)

    def isFinished(self) -> bool:
        # TODO
        return False
