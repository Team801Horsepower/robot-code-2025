from commands2 import CommandScheduler, Subsystem
from rev import SparkFlex, SparkFlexConfig
import rev
from wpimath import units
from wpilib import SmartDashboard
from math import pi

import config
from utils import clamp, time_f


class Wrist(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.wrist_motor = SparkFlex(
            config.wrist_motor_id, SparkFlex.MotorType.kBrushless
        )
        motor_config = SparkFlexConfig()
        motor_config.setIdleMode(SparkFlexConfig.IdleMode.kBrake)
        motor_config.closedLoop.pid(*config.wrist_pid_constants)
        motor_config.encoder.positionConversionFactor(2 * pi / config.wrist_gear_ratio)
        motor_config.encoder.velocityConversionFactor(
            2 * pi / 60 / config.wrist_gear_ratio
        )
        self.wrist_motor.configure(
            motor_config,
            SparkFlex.ResetMode.kResetSafeParameters,
            SparkFlex.PersistMode.kNoPersistParameters,
        )
        self.wrist_encoder = self.wrist_motor.getEncoder()
        self.wrist_pid = self.wrist_motor.getClosedLoopController()
        self.target_angle = config.wrist_neutral_angle
        self.tolerance = 0.05
        # Whether the elevator is high enough for the wrist to pivot downward
        self.passthrough_allowed = False

        self.set_angle(config.wrist_neutral_angle)

    def angle(self) -> float:
        return self.wrist_encoder.getPosition()

    def set_angle(self, angle: float):
        self.wrist_encoder.setPosition(angle)

    @time_f("periodic wrist")
    def periodic(self):
        # power = self.pid.calculate(
        #     self.angle(),
        #     clamp(self.lower_limit, config.wrist_limits[1], self.target_angle),
        # )
        # power = clamp(-0.5, 0.5, power)
        # self.wrist_motor.set(power)

        SmartDashboard.putNumber(
            "wrist pos",
            units.radiansToDegrees(self.angle()),
        )
        SmartDashboard.putNumber(
            "target wrist pos",
            units.radiansToDegrees(self.target_angle),
        )
        # SmartDashboard.putNumber("wrist power", power)

    def set_target(self, target_angle: float):
        self.target_angle = target_angle
        actual_target = clamp(self.lower_limit, config.wrist_limits[1], target_angle)
        self.wrist_pid.setReference(
            actual_target, rev.SparkLowLevel.ControlType.kPosition
        )

    @property
    def lower_limit(self) -> float:
        return (
            config.wrist_limits[0]
            if self.passthrough_allowed
            else config.wrist_neutral_angle + units.degreesToRadians(5)
        )

    def at_angle(self) -> bool:
        return abs(self.angle() - self.target_angle) <= self.tolerance

    def target_attainable(self) -> bool:
        return (
            config.wrist_limits[0] <= self.target_angle
            and self.target_angle <= config.wrist_limits[1]
        )
