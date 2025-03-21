from math import pi
from navx import AHRS
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from commands2 import CommandScheduler, Subsystem
from wpimath.units import inchesToMeters


from subsystems.chassis import Chassis
import config


class Odometry(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.ahrs = AHRS(AHRS.NavXComType.kMXP_SPI)
        self.translation = Translation2d()
        self.ahrs.setAngleAdjustment(0)
        self.angle_offset = Rotation2d()

    def rotation(self) -> Rotation2d:
        return Rotation2d.fromDegrees(-self.ahrs.getAngle()) + self.angle_offset

    def pose(self) -> Pose2d:
        return Pose2d(self.translation.x, self.translation.y, self.rotation())

    def reset(self, pose: Pose2d = Pose2d()):
        # self.translation = pose.translation()
        # self.ahrs.setAngleAdjustment(0)
        # self.ahrs.setAngleAdjustment(
        #     self.rotation().degrees() - pose.rotation().degrees()
        # )
        # self.ahrs.resetDisplacement()
        self.reset_translation(pose.translation())
        self.reset_heading(pose.rotation())

    def reset_translation(self, translation: Translation2d = Translation2d()):
        self.translation = translation

    def reset_heading(self, heading: Rotation2d = Rotation2d()):
        self.angle_offset += heading - self.rotation()

    def update(self, chassis: Chassis):
        count = 0
        total = Translation2d()

        for swerve in chassis.swerves:
            # Grab previous values and update immediately to prevent any loss of delta
            prev_drive = swerve.prev_drive_enc
            # prev_turn = swerve.prev_rotation
            swerve.update_prevs()

            delta = Translation2d(
                prev_drive - swerve.drive_encoder.getPosition(), 0
            ).rotateBy(swerve.rotation() + self.rotation())

            total += delta
            count += 1

        avg = total / count
        # self.translation += avg
        # Empirical scale factor lol
        self.translation += avg / 1.028099649

    def near_reef_dist(self, dist: float) -> bool:
        return (
            self.pose().translation() - config.flip_red(Translation2d(4.5, 4))
        ).norm() < dist  # from center of reef

    def near_reef_large(self) -> bool:
        return self.near_reef_dist(2.9)

    def near_reef(self) -> bool:
        return self.near_reef_dist(1.9)

    def near_source_dist(self, dist: float) -> bool:
        return (
            self.pose().translation() - config.flip_red(Translation2d(0.85, 0.65))
        ).norm() < dist or (
            self.pose().translation() - config.flip_red(Translation2d(0.85, 7.35))
        ).norm() < dist

    def near_source_large(self) -> bool:
        return self.near_source_dist(1.8)

    def near_source(self) -> bool:
        return self.near_source_dist(1.0)

    def near_targeted_stalk(self, stalk: int):
        # return (
        #     (
        #         self.pose().translation() -
        #         config.flip_red(Translation2d(4.5, 4)) +
        #                         Translation2d(inchesToMeters(32.75),
        #                             Rotation2d((pi if config.is_red() else 0)
        #                             + (pi/3)*int(stalk/2)))).norm() < 1.5
        # )
        return (
            self.pose().translation()
            - config.flip_red(Translation2d(4.5, 4))
            + Translation2d(
                inchesToMeters(32.75),
                Rotation2d((pi if config.is_red() else 0) + pi / 3 * int(stalk / 2)),
            )
        ).norm() < 1.5
