from navx import AHRS
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from commands2 import CommandScheduler, Subsystem


from subsystems.chassis import Chassis


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
        self.translation = pose.translation()
        self.ahrs.setAngleAdjustment(0)
        self.ahrs.setAngleAdjustment(
            self.rotation().degrees() - pose.rotation().degrees()
        )
        self.ahrs.resetDisplacement()
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
        self.translation += avg
