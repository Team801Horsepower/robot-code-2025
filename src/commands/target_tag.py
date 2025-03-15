from commands2 import Command
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from wpimath.geometry import Transform2d, Rotation2d, Translation2d
from wpimath.controller import PIDController
from wpimath import units
from wpilib import SmartDashboard
from math import pi, tan
from typing import Optional, Callable
from abc import abstractmethod

from subsystems.drive import Drive
from subsystems.vision import Vision

import config
from utils import clamp


# class TargetTag(Command, ABC):
class TargetTag(Command):
    def __init__(self):
        self.approach_pid = PIDController(5.5, 0, 0)
        self.strafe_pid = PIDController(5.5, 0, 0)
        self.theta_pid = PIDController(6.5, 0.0, 0.3)

        self.within_threshold = False

        # Camera frame yaw magnitude to use when determining when to
        # restrict rotation to keep the target april tag in frame
        # self.camera_bound_yaw = units.degreesToRadians(25)
        self.camera_bound_yaw = units.degreesToRadians(28)
        # P value with which to attempt to bound the rotation speed to keep the tag in frame
        self.camera_bound_p = 15.0

    @property
    @abstractmethod
    def drive(self) -> Drive:
        pass

    @property
    @abstractmethod
    def vision(self) -> Vision:
        pass

    @property
    @abstractmethod
    def tag_id(self) -> int:
        pass

    @property
    @abstractmethod
    def target_angle(self) -> float:
        pass

    @property
    @abstractmethod
    def left_target(self) -> float:
        pass

    @property
    @abstractmethod
    def right_target(self) -> float:
        pass

    @property
    @abstractmethod
    def left_i(self) -> int:
        pass

    @property
    @abstractmethod
    def right_i(self) -> int:
        pass

    @property
    @abstractmethod
    def get_left(self) -> Callable[[PhotonTrackedTarget], float]:
        pass

    @property
    @abstractmethod
    def get_right(self) -> Callable[[PhotonTrackedTarget], float]:
        pass

    @property
    @abstractmethod
    def use_diag(self) -> bool:
        pass

    def get_left_param(self) -> Optional[float]:
        return self.get_param(self.left_i, self.get_left)

    def get_right_param(self) -> Optional[float]:
        return self.get_param(self.right_i, self.get_right)

    def execute(self):
        rot = self.drive.odometry.rotation().radians()
        target_rot = self.target_angle
        while target_rot > rot + pi:
            target_rot -= 2 * pi
        while target_rot < rot - pi:
            target_rot += 2 * pi
        omega = self.theta_pid.calculate(rot, target_rot)

        left_param = self.get_left_param()
        right_param = self.get_right_param()

        if left_param is None or right_param is None:
            drive_speed = Translation2d(0, 0)
        else:
            SmartDashboard.putNumber("left param", left_param)
            SmartDashboard.putNumber("right param", right_param)
            SmartDashboard.putNumber("left target", self.left_target)
            SmartDashboard.putNumber("right target", self.right_target)

            strafe_speed = self.strafe_pid.calculate(
                left_param + right_param, self.left_target + self.right_target
            )

            diff = left_param - right_param
            target_diff = self.left_target - self.right_target
            approach_speed = self.approach_pid.calculate(diff, target_diff)

            # TODO: test this; not sure if the tan needs to be added or subtracted
            if self.use_diag:
                diag_angle = (self.left_target + self.right_target) / 2
                strafe_speed += approach_speed * tan(diag_angle)

            SmartDashboard.putNumber("strafe speed", strafe_speed)
            SmartDashboard.putNumber("approach speed", approach_speed)

            drive_speed = Translation2d(approach_speed, strafe_speed)

            SmartDashboard.putNumber(
                "tt sum", units.radiansToDegrees(left_param + right_param)
            )
            SmartDashboard.putNumber(
                "tt sum target",
                units.radiansToDegrees(self.left_target + self.right_target),
            )
            SmartDashboard.putNumber(
                "tt diff", units.radiansToDegrees(left_param - right_param)
            )
            SmartDashboard.putNumber(
                "tt diff target",
                units.radiansToDegrees(self.left_target - self.right_target),
            )

            # Restrict rotation if it would cause the tag to exit the frame
            # TODO: only use this for yaw angles, not pitch angles
            # omega_min = self.camera_bound_p * max(
            #     -self.camera_bound_yaw - left_param,
            #     -self.camera_bound_yaw - right_param,
            # )
            # omega_max = self.camera_bound_p * min(
            #     self.camera_bound_yaw - left_param,
            #     self.camera_bound_yaw - right_param,
            # )
            # SmartDashboard.putNumber("omega min", omega_min)
            # SmartDashboard.putNumber("omega max", omega_max)
            # omega = clamp(omega_min, omega_max, omega)

            sum_within_threshold = abs(
                left_param
                + right_param
                - (self.left_target + self.right_target)
                # ) < units.degreesToRadians(1.0)
            ) < units.degreesToRadians(1.5)
            diff_within_threshold = abs(
                left_param
                - right_param
                - (self.left_target - self.right_target)
                # ) < units.degreesToRadians(1.0)
            ) < units.degreesToRadians(1.5)
            self.within_threshold = sum_within_threshold and diff_within_threshold

        # max_drive_speed = config.auto_drive_speed
        # max_drive_speed = 2
        max_drive_speed = 2.5

        norm = drive_speed.norm()
        if norm > max_drive_speed:
            drive_speed = drive_speed * max_drive_speed / norm
        if abs(omega) > max_drive_speed:
            omega = omega * config.auto_turn_speed / abs(omega)

        SmartDashboard.putNumber("omega", omega)

        if self.within_threshold:
            drive_input = Transform2d()
        else:
            drive_input = Transform2d(drive_speed, Rotation2d(omega))

        self.drive.drive(drive_input)

    def isFinished(self) -> bool:
        return self.within_threshold

    # def get_yaw(self, i: int) -> Optional[float]:
    #     targets = self.vision.results[i].getTargets()
    #     for target in targets:
    #         if target.getFiducialId() == self.tag_id:
    #             return (
    #                 units.degreesToRadians(target.getYaw())
    #                 # Approximately correct for heading error
    #                 # TODO: this correction element is not tested
    #                 # - self.drive.odometry.rotation().radians()
    #                 # + self.target_angle
    #             )
    #     return None

    def get_param(self, i, f) -> Optional[float]:
        targets = self.vision.results[i].getTargets()
        for target in targets:
            if target.getFiducialId() == self.tag_id:
                return units.degreesToRadians(f(target))
        return None

    def end(self, interrupted: bool):
        self.drive.drive(Transform2d())
