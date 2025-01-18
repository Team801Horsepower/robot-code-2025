from subsystems.swerve import Swerve
import config

from math import pi
from functools import reduce
from typing import Tuple

import rev
from rev import SparkMax, SparkFlex, SparkBaseConfig, SparkBase
from wpimath.geometry import Translation2d, Transform2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
from phoenix6.hardware.cancoder import CANcoder


# pylint: disable=missing-docstring, too-few-public-methods
class Chassis:
    def __init__(self) -> None:
        def make_swerve(t: Tuple[int, Tuple[int, int, int]]) -> Swerve:
            i, (drive_id, turn_id, cancoder_id) = t
            positional_offset = [-1 / 4, 1 / 2, 0, 1 / 4][i]
            return Swerve(
                SparkFlex(drive_id, SparkMax.MotorType.kBrushless),
                SparkMax(turn_id, SparkMax.MotorType.kBrushless),
                CANcoder(cancoder_id),
                positional_offset,
            )

        self.swerves = list(
            map(make_swerve, enumerate([config.swerves[i] for i in config.swerve_ids]))
        )

        for swerve in self.swerves:
            drive_config = (
                SparkBaseConfig()
                .inverted(False)
                .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
            )
            drive_config.encoder.velocityConversionFactor(
                pi * config.wheel_diameter / 60 / config.drive_gear_ratio
            ).positionConversionFactor(
                pi * config.wheel_diameter / config.drive_gear_ratio
            )
            drive_config.closedLoop.pid(*config.drive_pid)
            swerve.drive_motor.configure(
                drive_config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters,
            )
            swerve.drive_encoder.setPosition(0.0)

            turn_config = (
                SparkBaseConfig()
                .inverted(False)
                .setIdleMode(SparkBaseConfig.IdleMode.kCoast)
            )
            turn_config.encoder.positionConversionFactor(
                2 * pi / config.turn_gear_ratio
            )
            turn_config.closedLoop.pid(*config.turn_pid)
            swerve.turn_motor.configure(
                turn_config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters,
            )

            # Prevs must be updated again after conversion factors were changed
            swerve.update_prevs()

    def set_swerves(self):
        for swerve in self.swerves:
            swerve.reset_from_abs_enc()

    def zero_swerves(self):
        self.set_swerves()
        for swerve in self.swerves:
            swerve.turn_pid.setReference(0.0, rev.SparkLowLevel.ControlType.kPosition)

    def drive(self, vel: Transform2d) -> None:
        """
        `vel`: (forward, leftward, counterclockwise), m/s, rad/s
        """
        # HACK: We don't know the actual reason for the discrepancy
        #       between configured speeds and effective speeds.
        # TODO: Check if we still need this on the new chassis
        # vel *= 2.5

        # Equality check is fine here since the deadzone handles rounding to 0.
        if vel.rotation().degrees() == 0.0 and vel.translation().norm() == 0.0:
            for swerve in self.swerves:
                swerve.drive_pid.setReference(
                    0.0, rev.SparkLowLevel.ControlType.kVelocity
                )
            return

        swerves = zip(
            self.swerves,
            [(1, 1), (-1, 1), (1, -1), (-1, -1)],
        )
        for swerve, pos in swerves:
            # Normalizing involves dividing by (radius * 2), but converting from
            # angular velocity to linear velocity means multiplying by radius,
            # leaving only a division by 2.
            rot_vec = config.robot_dimensions / 2.0 * vel.rotation().radians()
            # Negate components according to coordinates and rotate 90° counterclockwise.
            rot_vec = Translation2d(-rot_vec.y * pos[1], rot_vec.x * pos[0])

            total_vec = rot_vec + vel.translation()

            turn_angle = total_vec.angle().radians()
            drive_speed = total_vec.norm()

            cur_position = swerve.turn_encoder.getPosition()

            while turn_angle < cur_position - pi:
                turn_angle += 2.0 * pi
            while turn_angle > cur_position + pi:
                turn_angle -= 2.0 * pi

            # Flip check
            quarter_turn = pi / 2.0
            if turn_angle < cur_position - quarter_turn:
                turn_angle += pi
                drive_speed *= -1.0
            elif turn_angle > cur_position + quarter_turn:
                turn_angle -= pi
                drive_speed *= -1.0

            swerve.turn_pid.setReference(
                turn_angle, rev.SparkLowLevel.ControlType.kPosition
            )
            swerve.drive_pid.setReference(
                -drive_speed, rev.SparkLowLevel.ControlType.kVelocity
            )

    # Robot relative
    def chassis_speeds(self) -> ChassisSpeeds:
        def hadamard(a: Translation2d, b: Tuple[float, float]) -> Translation2d:
            return Translation2d(a.x * b[0], a.y * b[1])

        def cross(a: Translation2d, b: Translation2d) -> float:
            return a.x * b.y - a.y * b.x

        movs = []
        rots = []
        swerves = zip(
            self.swerves,
            [(1, 1), (-1, 1), (1, -1), (-1, -1)],
        )
        for swerve, pos in swerves:
            v = (
                Translation2d(1, 0).rotateBy(
                    Rotation2d.fromDegrees(swerve.turn_encoder.getPosition() * 180 / pi)
                )
                * swerve.drive_encoder.getVelocity()
            )
            d = config.robot_dimensions
            dnorm = d.norm()

            rot = 2 * cross(v, hadamard(d / dnorm, pos)) / dnorm
            rots.append(rot)

            movs.append(v)

        mov = reduce(Translation2d.__add__, movs) / len(movs)
        rot = sum(rots) / len(rots)

        return ChassisSpeeds(-mov.x, -mov.y, rot)
