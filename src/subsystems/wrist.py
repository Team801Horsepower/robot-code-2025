from commands2 import CommandScheduler, Subsystem
from rev import SparkFlex, SparkFlexConfig
from wpimath.controller import PIDController
from wpimath import units
from wpilib import SmartDashboard

import config
from utils import clamp


class Wrist(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.wrist_motor = SparkFlex(
            config.wrist_motor_id, SparkFlex.MotorType.kBrushless
        )
        motor_config = SparkFlexConfig()
        motor_config.setIdleMode(SparkFlexConfig.IdleMode.kBrake)
        self.wrist_motor.configure(
            motor_config,
            SparkFlex.ResetMode.kResetSafeParameters,
            SparkFlex.PersistMode.kNoPersistParameters,
        )
        self.wrist_encoder = self.wrist_motor.getEncoder()
        self.wrist_encoder.setPosition(
            config.wrist_neutral_angle * config.wrist_gear_ratio
        )
        self.pid = PIDController(
            *config.wrist_pid_constants
        )  # TODO: Change to actual constants
        self.target_angle = config.wrist_neutral_angle
        self.tolerance = 0.05
        # Whether the elevator is high enough for the wrist to pivot downward
        self.passthrough_allowed = False

        scheduler.registerSubsystem(self)

    def angle(self) -> float:
        return self.wrist_encoder.getPosition() / config.wrist_gear_ratio

    def set_angle(self, angle: float):
        self.wrist_encoder.setPosition(angle * config.wrist_gear_ratio)

    def periodic(self):
        power = self.pid.calculate(
            self.angle(),
            clamp(self.lower_limit, config.wrist_limits[1], self.target_angle),
        )
        self.wrist_motor.set(power)
        SmartDashboard.putNumber("wrist encoder", self.wrist_encoder.getPosition())

    @property
    def lower_limit(self) -> float:
        return (
            config.wrist_limits[0]
            if self.passthrough_allowed
            else config.wrist_neutral_angle + units.degreesToRadians(5)
        )

    def fold(self):
        self.target = 0.0  # TODO: change to preferred folding position

    def at_angle(self) -> bool:
        return abs(self.angle() - self.target_angle) <= self.tolerance

    def target_attainable(self) -> bool:
        return (
            config.wrist_limits[0] <= self.target_angle
            and self.target_angle <= config.wrist_limits[1]
        )
