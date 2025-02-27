from commands2 import CommandScheduler, Subsystem
from rev import SparkFlex
from wpilib import DigitalInput

import config


class Claw(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.motor = SparkFlex(config.claw_motor_id, SparkFlex.MotorType.kBrushless)
        self.sensor = DigitalInput

    def periodic(self):
        pass

    def run(self, power: float):
        self.motor.set(power)

    def has_coral(self) -> bool:
        return not self.sensor.get()
