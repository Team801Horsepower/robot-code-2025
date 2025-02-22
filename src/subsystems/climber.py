from commands2 import CommandScheduler, Subsystem
from rev import SparkMax, SparkFlex, SparkBaseConfig, SparkBase
import config

class Climber(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)
        self.climb_motor = SparkFlex(config.climb_motor_id, SparkFlex.MotorType.kBrushless)

    def periodic(self):
        pass

    def run(self, power: float):
        self.climb_motor.set(power)
