from commands2 import CommandScheduler, Subsystem
from rev import SparkFlex, SparkFlexConfig
from wpimath.controller import PIDController
from wpilib import SmartDashboard

from config import (
    wrist_motor_id,
    wrist_pid_constants,
    wrist_limits,
    wrist_neutral_angle,
    wrist_gear_ratio,
)
from utils import clamp


class Wrist(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.wrist_motor = SparkFlex(wrist_motor_id, SparkFlex.MotorType.kBrushless)
        config = SparkFlexConfig()
        config.setIdleMode(SparkFlexConfig.IdleMode.kBrake)
        self.wrist_motor.configure(
            config,
            SparkFlex.ResetMode.kResetSafeParameters,
            SparkFlex.PersistMode.kNoPersistParameters,
        )
        self.wrist_encoder = self.wrist_motor.getEncoder()
        self.wrist_encoder.setPosition(wrist_neutral_angle * wrist_gear_ratio)
        self.pid = PIDController(
            *wrist_pid_constants
        )  # TODO: Change to actual constants
        self.target_angle = wrist_neutral_angle
        self.tolerance = 0.05
        scheduler.registerSubsystem(self)

    def angle(self) -> float:
        return self.wrist_encoder.getPosition() / wrist_gear_ratio

    def set_angle(self, angle: float):
        self.wrist_encoder.setPosition(angle * wrist_gear_ratio)

    def periodic(self):
        self.wrist_motor.set(
            self.pid.calculate(
                self.angle(),
                clamp(wrist_limits[0], wrist_limits[1], self.target_angle),
            )
        )
        SmartDashboard.putNumber("wrist encoder", self.wrist_encoder.getPosition())

    def fold(self):
        self.target = 0.0  # TODO: change to preferred folding position

    def at_angle(self) -> bool:
        return abs(self.angle() - self.target_angle) <= self.tolerance

    def target_attainable(self) -> bool:
        return (
            wrist_limits[0] <= self.target_angle
            and self.target_angle <= wrist_limits[1]
        )
