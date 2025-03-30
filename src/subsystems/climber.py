from commands2 import CommandScheduler, Subsystem
from rev import SparkFlex, SparkBaseConfig, SparkBase
from wpilib import SmartDashboard

import config


class Climber(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

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

    def periodic(self):
        config.climb_power_mult = SmartDashboard.getNumber(
            "climb power multiplier", config.climb_power_mult
        )

    def climb(self, power: float):
        self.motor.set(power)
