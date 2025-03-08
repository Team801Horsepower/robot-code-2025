from commands2 import CommandScheduler, Subsystem
from rev import SparkFlex, SparkFlexConfig
from wpilib import AnalogInput, DigitalInput, SmartDashboard
import time

import config


class Claw(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        self.motor = SparkFlex(config.claw_motor_id, SparkFlex.MotorType.kBrushless)
        motor_config = SparkFlexConfig().inverted(True)
        motor_config.setIdleMode(SparkFlexConfig.IdleMode.kBrake)
        self.motor.configure(
            motor_config,
            SparkFlex.ResetMode.kResetSafeParameters,
            SparkFlex.PersistMode.kNoPersistParameters,
        )
        self.algae_sensor = AnalogInput(0)
        self.coral_sensor = DigitalInput(1)
        scheduler.registerSubsystem(self)

        self.coral_detected_time = time.time()
        self.algae_detected_time = time.time()

    def periodic(self):
        if not self.coral_detected():
            self.coral_detected_time = time.time()
        if not self.algae_detected():
            self.algae_detected_time = time.time()

        SmartDashboard.putNumber("algae sensor value", self.algae_sensor.getValue())
        SmartDashboard.putBoolean("algae detected", self.algae_detected())
        SmartDashboard.putBoolean("coral detected", self.coral_detected())

        SmartDashboard.putBoolean("has coral", self.has_coral())

    def set(self, power: float):
        if self.has_coral():
            power = max(power, 0)
        self.motor.set(power)

    def algae_detected(self) -> bool:
        return self.algae_sensor.getValue() > 1300

    def coral_detected(self) -> bool:
        return not self.coral_sensor.get()

    def has_coral(self) -> bool:
        return time.time() - self.coral_detected_time > 0.025

    def has_algae(self) -> bool:
        return time.time() - self.algae_detected_time > 0.025
