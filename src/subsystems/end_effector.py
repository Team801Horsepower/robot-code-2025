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
        self.has_algae = False
        self.has_coral = False
        

    def periodic(self):
        encoder_value = self.wrist_encoder.getPosition()
        self.wrist_motor.set(self.pid_yippee.calculate(encoder_value, self.target))
    
    def spin(self, power):
        self.intake_motor.set(power)
    
    def stopSpin(self):
        self.intake_motor.set(0)
        
    def setTarget(self, target):
        self.target = target
        
    #TODO: ensure that everything is the right direction (negatives)
    
    def fold(self):
        self.setTarget(self, 0)
        self.stopSpin(self)
        
    def getAlgae(self):
        algae_position = 1.3 #TODO: change this to the actual position required
        self.setTarget(self, algae_position)
        if self.has_algae == False:
            self.spin(self, 1)
        else:
            self.spin(self, 0.5) #TODO: find the ideal waiting motor power
    
    def processorAlgae(self):
        algae_processor_position = 0.4 #TODO: change this to the actual position required
        self.setTarget(self, algae_processor_position)
        if self.has_algae == True:
            self.spin(self, -1)
        else:
            self.spin(self, -0.5)

    def bargeAlgae(self): #WARNING: SINCE THE BARGE COULD BE SCORED FROM TWO DIFFERENT POSITIONS, THIS COULD BE IN FLUX
        barge_position = 2.4 #TODO: change this to the actual position required
        self.setTarget(self, barge_position)
        if self.has_algae == True:
            self.spin(self, -1)
        else:
            self.spin(self, -0.5)
    
    def getCoral(self):
        coral_human_position = 0.5 #TODO: change this to the actual position required
        self.setTarget(self, coral_human_position)
        if self.has_coral == False:
            self.spin(self, -1)
        else:
            self.spin(self, -0.5)
    
    def level1(self):
        level1_position = 1.6 #TODO: change this to correct lvl1 position
        self.setTarget(self, level1_position)
        if self.has_coral == True:
            self.spin(self, 1)
        else:
            self.spin(self, 0.5)
    
    def level2Or3(self):
        level2_3_position = 1.9 #TODO: change this to correct lvl2/lvl3 position
        self.setTarget(self, level2_3_position)
        if self.has_coral == True:
            self.spin(self, 1)
        else:
            self.spin(self, 0.5)
    
    def level4(self):
        level4_position = 2.6 #TODO: change this to correct lvl4 position
        self.setTarget(self, level4_position)
        if self.has_coral == True:
            self.spin(self, 1)
        else:
            self.spin(self, 0.5)