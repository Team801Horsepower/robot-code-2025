from commands2 import CommandScheduler, Subsystem
from phoenix6.hardware.cancoder import CANcoder
from rev import SparkFlex, SparkBaseConfig
from math import sin
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
import config


class Elevator(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        # State variables
        self.setpoint_length = 1.0  # meters
        self.setpoint_theta = 0.5  # radians
        self.length = 1.0  # meters
        self.theta = 0.5  # radians

        # Initialize pivot motors
        self.pivot_motors = [
            SparkFlex(motor_id, SparkFlex.MotorType.kBrushless)
            for motor_id in config.pivot_motor_ids
        ]

        # Configure pivot motors
        for i, motor in enumerate(self.pivot_motors):
            config = SparkBaseConfig()
            config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
            config.setInverted(i % 2 == 1)  # Alternate inverting for correct rotation
            motor.applyConfig(config)

        # Initialize extension motors
        self.extension_motors = [
            SparkFlex(motor_id, SparkFlex.MotorType.kBrushless)
            for motor_id in config.extension_motor_ids
        ]

        # Encoders
        self.pivot_encoder = CANcoder(config.pivot_encoder_id)
        self.extension_encoder = CANcoder(config.extension_encoder_id)

        # PID Controller for pivot movement with motion profiling
        self.theta_pid = ProfiledPIDController(
            kp=2.0, ki=0.5, kd=0.025,
            constraints=TrapezoidProfile.Constraints(maxVelocity=1.0, maxAcceleration=1.0)
        )

        self.length_pid = ProfiledPIDController(
            kp=1.0, ki=0.2, kd=0.05, 
            constraints=TrapezoidProfile.Constraints(maxVelocity=0.5, maxAcceleration=0.5)
        )


    def periodic(self):
        """Runs periodically in the scheduler (for feedback control)."""
        self.target_setpoint()

    def target_setpoint(self):
        """Controls pivot motors to reach the desired angle using PID & gravity compensation."""
        output = self.gravity_torque() + self.theta_pid.calculate(self.get_theta(), self.setpoint_theta)
        
        for motor in self.pivot_motors:
            motor.set(output)

    def get_length(self) -> float:
        """Returns the elevator's extension length in meters."""
        return self.extension_encoder.get_absolute_position() * config.extension_encoder_ratio

    def get_theta(self) -> float:
        """Returns the current pivot angle in radians."""
        return self.pivot_encoder.get_absolute_position() * config.pivot_encoder_ratio

    def calculate_moi(self) -> float:
        """Calculates and returns the moment of inertia (MOI) of the elevator."""
        return self.length ** 2 * ((config.elevator_mass / 3) + config.end_effector_mass)

    def gravity_torque(self) -> float:
        """Computes the torque required to counteract gravity."""
        return sin(self.theta) * 9.81 * (config.elevator_mass / 2 + config.end_effector_mass) * self.length

