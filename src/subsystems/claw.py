from commands2 import CommandScheduler, Subsystem
from rev import SparkFlex, SparkFlexConfig
from wpilib import AnalogInput, DigitalInput, SmartDashboard
import time

import config
from utils import clamp


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

        self.coral_detected_time = None
        self.algae_detected_time = None

    def periodic(self):
        if not self.coral_detected():
            self.coral_detected_time = None
        elif self.coral_detected_time is None:
            self.coral_detected_time = time.time()
        if not self.algae_detected():
            self.algae_detected_time = None
        elif self.algae_detected_time is None:
            self.algae_detected_time = time.time()

        SmartDashboard.putNumber("algae sensor value", self.algae_sensor.getValue())
        SmartDashboard.putBoolean("algae detected", self.algae_detected())
        SmartDashboard.putBoolean("coral detected", self.coral_detected())

        SmartDashboard.putBoolean("has coral", self.has_coral())
        SmartDashboard.putBoolean("has algae", self.has_algae())

    def set(self, power: float):
        if self.has_coral():
            power = max(power, 0)
        power = clamp(-0.75, 1, power)
        self.motor.set(power)

    def algae_detected(self) -> bool:
        return self.algae_sensor.getValue() > 1300

    def coral_detected(self) -> bool:
        return not self.coral_sensor.get()

    # FIXME: This gives a false positive if the loop time overruns the threshold.
    #        Maybe instead of setting it to the current time every frame, we set
    #        it to None by default, then when coral_detected() is true, we set
    #        the time value only if it's currently None, and leave it alone
    #        otherwise. If coral_detected() is every seen to be False, we set
    #        it back to None. Then, has_coral() is a check of first,
    #        coral_detected_time is not None, and then that the elapsed since then
    #        is greater than the threshold.
    def has_coral(self) -> bool:
        # return time.time() - self.coral_detected_time > 0.025
        return self.coral_detected_time is not None and time.time() - self.coral_detected_time > 0.025

    def has_algae(self) -> bool:
        return self.algae_detected_time is not None and time.time() - self.algae_detected_time > 0.025
