from commands2 import CommandScheduler, Subsystem
from phoenix6.hardware.cancoder import CANcoder
from rev import SparkMax, SparkFlex, SparkBaseConfig, SparkBase
from math import sin
from wpimath.controller import PIDController
import config


class Elevator(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)
        self.setpoint_length = 1 # m
        self.setpoint_theta = 0.5 # radian

        self.length = 1 # m
        self.theta = 0.5 # radian

        self.pivot_motors = [SparkFlex(id, SparkFlex.MotorType.kBrushless) for id in config.pivot_motor_ids]
        self.extension_motors = [SparkFlex(id, SparkFlex.MotorType.kBrushless) for id in config.extension_motor_ids]

        self.pivot_encoder = CANcoder(config.pivot_encoder_id)
        self.extension_encoder = CANcoder(config.extension_encoder_id)

        self.theta_pid = PIDController(1, 0, 1)

    def periodic(self):
        self.get_length()
        self.get_theta()
        self.target_setpoint()

    def target_setpoint(self):
        output = self.gravity_torque() + self.theta_pid.calculate(self.get_theta(), self.setpoint_theta)
        for motor in self.pivot_motors:
            motor.set(output)

    def get_length(self):
        return self.extension_encoder.get_absolute_position() * config.extension_encoder_ratio
    
    def get_theta(self):
        return self.pivot.get_absolute_position() * config.pivot_encoder_ratio

    def calculate_moi(self):
        return self.length() ** 2 * ((config.elevator_mass / 3) + config.end_effector_mass)

    def gravity_torque(self):
        return sin(self.angle) * 9.81 * self.calculate_moi()


