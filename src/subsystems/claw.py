from commands2 import CommandScheduler, Subsystem
from rev import SparkFlex, SparkFlexConfig
from wpilib import DigitalInput
from config import claw_motor_id


class Claw(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        self.motor = SparkFlex(claw_motor_id, SparkFlex.MotorType.kBrushless)
        config = SparkFlexConfig()
        config.setIdleMode(SparkFlexConfig.IdleMode.kBrake)
        self.motor.configure(
            config,
            SparkFlex.ResetMode.kResetSafeParameters,
            SparkFlex.PersistMode.kNoPersistParameters,
        )
        self.sensor = DigitalInput(1)
        scheduler.registerSubsystem(self)

    def periodic(self):
        pass

    def gather(self, power: float):
        self.motor.set(power)

    def has_coral(self) -> bool:
        return not self.sensor.get()
