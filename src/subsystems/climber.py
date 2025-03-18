from commands2 import CommandScheduler, Subsystem
from rev import SparkMax, SparkFlex, SparkBaseConfig, SparkBase
from wpilib import DigitalInput, SmartDashboard
import config


class Climber(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)
        self.climb_motor = SparkFlex(
            config.climb_motor_id, SparkFlex.MotorType.kBrushless
        )
        self.induction_sensor = DigitalInput(2)

    def periodic(self):
        SmartDashboard.putBoolean("climbed", self.climbed())

    def climb(self, power: float):
        if not self.climbed():
            self.climb_motor.set(-power)
        else:
            self.climb_motor.set(0)

    def climbed(self) -> bool:
        return not self.induction_sensor.get()
