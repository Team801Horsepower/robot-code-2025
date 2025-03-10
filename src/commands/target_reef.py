from commands2 import Command
from wpimath.geometry import Transform2d, Rotation2d, Translation2d
from wpimath.controller import PIDController
from wpimath import units
from wpilib import SmartDashboard
from math import pi, tan
from typing import Optional

from subsystems.drive import Drive
from subsystems.vision import Vision

import config


class TargetReef(Command):
    def __init__(self, drive: Drive, vision: Vision, stalk_i: int):
        self.drive = drive
        self.vision = vision

        tag_ids = [10, 11, 6, 7, 8, 9] if config.is_red() else [21, 20, 19, 18, 17, 22]
        face_i = int(stalk_i / 2)
        self.tag_id = tag_ids[face_i]
        self.target_angle = 0 if config.is_red() else pi
        self.target_angle += pi / 3 * face_i

        self.approach_pid = PIDController(15.0, 0, 0)
        self.strafe_pid = PIDController(10.0, 0, 0)
        # TODO: why is kI set to 5?
        self.theta_pid = PIDController(6.0, 5.0, 0.3)

        self.within_threshold = False

        # a = units.degreesToRadians(11.66)
        # b = units.degreesToRadians(12.11)
        # a = units.degreesToRadians(11.37)
        # b = units.degreesToRadians(10.35)
        # a = units.degreesToRadians(12.69)
        # b = units.degreesToRadians(10.04)
        # a = units.degreesToRadians(0)
        # b = units.degreesToRadians(13.9)
        left_stalk = stalk_i % 2 == 0
        # self.left_target = a if left_stalk else -b
        # self.right_target = b if left_stalk else -a

        self.left_target = units.degreesToRadians(13.6 if left_stalk else -12.11)
        self.right_target = units.degreesToRadians(13.75 if left_stalk else -11.66)

    def execute(self):
        # new_p = SmartDashboard.getNumber("approach P", 15.0)
        # self.approach_pid.setP(new_p)

        rot = self.drive.odometry.rotation().radians()
        target_rot = self.target_angle
        while target_rot > rot + pi:
            target_rot -= 2 * pi
        while target_rot < rot - pi:
            target_rot += 2 * pi
        omega = self.theta_pid.calculate(rot, target_rot)
        if abs(omega) > config.auto_turn_speed:
            omega = omega * config.auto_turn_speed / abs(omega)

        left_yaw = self.get_yaw(0)
        right_yaw = self.get_yaw(2)

        if left_yaw is None or right_yaw is None:
            drive_speed = Translation2d(0, 0)
        else:
            SmartDashboard.putNumber("left yaw", left_yaw)
            SmartDashboard.putNumber("right yaw", right_yaw)
            SmartDashboard.putNumber("left target", self.left_target)
            SmartDashboard.putNumber("right target", self.right_target)

            strafe_speed = self.strafe_pid.calculate(
                left_yaw + right_yaw, self.left_target + self.right_target
            )

            diff = left_yaw - right_yaw
            target_diff = self.left_target - self.right_target
            approach_speed = self.approach_pid.calculate(diff, target_diff)

            # TODO: test this; not sure if the tan needs to be added or subtracted
            diag_angle = (self.left_target + self.right_target) / 2
            strafe_speed += approach_speed * tan(diag_angle)

            SmartDashboard.putNumber("strafe speed", strafe_speed)
            SmartDashboard.putNumber("approach speed", approach_speed)

            drive_speed = Translation2d(approach_speed, strafe_speed)

            SmartDashboard.putNumber("tr sum", left_yaw + right_yaw)
            SmartDashboard.putNumber(
                "tr sum target", self.left_target + self.right_target
            )
            SmartDashboard.putNumber("tr diff", left_yaw - right_yaw)
            SmartDashboard.putNumber(
                "tr diff target", self.left_target - self.right_target
            )

            sum_within_threshold = abs(
                left_yaw + right_yaw - (self.left_target + self.right_target)
            ) < units.degreesToRadians(2)
            diff_within_threshold = abs(
                left_yaw - right_yaw - (self.left_target - self.right_target)
            ) < units.degreesToRadians(2)
            self.within_threshold = sum_within_threshold and diff_within_threshold

        norm = drive_speed.norm()
        if norm > config.auto_drive_speed:
            drive_speed = drive_speed * config.auto_drive_speed / norm

        if self.within_threshold:
            drive_input = Transform2d()
        else:
            drive_input = Transform2d(drive_speed, Rotation2d(omega))

        self.drive.drive(drive_input)

    def isFinished(self) -> bool:
        # TODO
        return self.within_threshold

    def get_yaw(self, i: int) -> Optional[float]:
        targets = self.vision.results[i].getTargets()
        for target in targets:
            if target.getFiducialId() == self.tag_id:
                return units.degreesToRadians(target.getYaw())
        return None
