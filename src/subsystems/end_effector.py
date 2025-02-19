from commands2 import CommandScheduler, Subsystem
from rev import SparkMax, SparkFlex, SparkBaseConfig, SparkBase
from wpimath.controller import PIDController
import config

# Eleanor, Hudson, and Yilu - 1/20/2025
class EndEffector(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)
        
        self.intake_motor = SparkFlex(config.effector_motor_id, SparkFlex.MotorType.kBrushless)
        self.intake_encoder = self.intake_motor.getEncoder()
        self.wrist_motor = SparkFlex(config.wrist_motor_id, SparkFlex.MotorType.kBrushless)
        self.wrist_encoder = self.wrist_motor.getEncoder()
        self.pid_yippee = PIDController(0, 0, 0.5) #TODO: Change to actual constants
        self.position = 0 #can change, don't care. It's in radians?
        self.target = 0
        

    def periodic(self):
        encoder_value = self.wrist_encoder.getPosition()
        self.wrist_motor.set(self.pid_yippee.calculate(encoder_value, self.target))
    
    def spin(self, power):
        self.intake_motor.set(power)
    
    def stopSpin(self):
        self.intake_motor.set(0)
        
    def setTarget(self, target):
        self.target = target