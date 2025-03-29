from commands2 import CommandScheduler, Subsystem
from rev import SparkMax, SparkFlex, SparkBaseConfig, SparkBase
from wpilib import SmartDashboard
import config


class Climber(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)
        self.climb_motor = SparkFlex(
            config.climb_motor_id, SparkFlex.MotorType.kBrushless
        )
        # self.cage_gathered = False

    def periodic(self):
        pass
        # SmartDashboard.putBoolean("climbed", self.climbed())

        # # SmartDashboard.putNumber(
        #         "Climb Motor Current Draw",
        #         self.climb_motor.getOutputCurrent(),
        #     )
        # if self.climb_motor.getOutputCurrent() > 801: # TODO: add real threshold :P
        #     self.cage_gathered = True

        # self.cage_gathered = True

    def climb(self, power: float):
        # if not self.climbed():
        #     self.climb_motor.set(-power)
        # else:
        #     self.climb_motor.set(0)
        self.climb_motor.set(-power)

    # def climbed(self) -> bool:
    #     return self.cage_gathered
        
