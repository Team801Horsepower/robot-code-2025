from commands2 import CommandScheduler, Subsystem
from rev import SparkMax, SparkFlex, SparkBaseConfig, SparkBase
from wpimath.controller import PIDController
import config


class Wrist(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.wrist_motor = SparkFlex(
            config.wrist_motor_id, SparkFlex.MotorType.kBrushless
        )
        self.wrist_encoder = self.wrist_motor.getEncoder()
        self.pid = PIDController(0.5, 0, 0)  # TODO: Change to actual constants
        self.target_angle = 0.0
        self.tolerance = 0.05

    def position(self) -> float:
        return self.wrist_encoder.getPosition()

    def periodic(self):
        self.wrist_motor.set(self.pid.calculate(self.position(), min(max(self.target_angle,config.wrist_limits[0]), config.wrist_limits[1])))

    def fold(self):
        self.target = 0.0  # TODO: change to preferred folding position

    def at_angle(self) -> bool:
        return abs(self.position() - self.target_angle) <= self.tolerance

    def target_attainable(self) -> bool:
        return config.wrist_limits[0] <= self.target_angle and self.target_angle <= config.wrist_limits[1]
