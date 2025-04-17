from commands2 import CommandScheduler, Subsystem
from rev import SparkFlex, SparkBaseConfig, SparkBase
from wpilib import SmartDashboard

import config


class Climber(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.current_marker = 0.0
        self.input_marker = 0.0
        self.input = 0.0
        self.last_input = 0.0

        self.motor = SparkFlex(config.climb_motor_id, SparkFlex.MotorType.kBrushless)
        motor_config = (
            SparkBaseConfig()
            .inverted(True)
            .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        )
        self.motor.configure(
            motor_config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters,
        )

        SmartDashboard.putNumber("climb power multiplier", config.climb_power_mult)
        SmartDashboard.putNumber(
            "climb power large multiplier", config.climb_power_mult_when_low
        )

    def periodic(self):
        config.climb_power_mult = SmartDashboard.getNumber(
            "climb power multiplier", config.climb_power_mult
        )
        config.climb_power_mult_when_low = SmartDashboard.getNumber(
            "climb power large multiplier", config.climb_power_mult_when_low
        )
        SmartDashboard.putNumber("Current Marker", self.current_marker)
        SmartDashboard.putNumber("Climb Current", self.motor.getOutputCurrent())
        SmartDashboard.putNumber("Ajusted Climb Marker", self.current_marker - config.input_multiplier * max(0, abs(self.input - self.last_input)))
        

    def climb(self, power: float):
        self.motor.set(power)
        self.last_input = self.input
        self.input = power

    def cage_gathered(self):
        self.current_marker = (self.current_marker + config.current_multiplier * self.motor.getOutputCurrent()) / (1 + config.current_multiplier)
        self.input_marker = (self.input_marker + config.input_multiplier * max(0, abs(self.input - self.last_input))) / (1 + config.input_multiplier)

        if self.current_marker - config.cage_current_threshold * self.input_marker > config.cage_current_threshold:
            return True
        return False
