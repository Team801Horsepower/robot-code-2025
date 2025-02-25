from commands2 import CommandScheduler, Subsystem
from wpilib import DutyCycleEncoder
from wpimath.controller import ProfiledPIDController
from math import pi
from wpimath.trajectory import TrapezoidProfile
from rev import SparkFlex, SparkBaseConfig


from subsystems.elevator import Elevator
from config import (
    pivot_motor_ids,
    pivot_pid_constants,
    pivot_pid_constraint_constants,
    pivot_encoder_id,
    pivot_angle_offset,
    pivot_epsilon_pos,
    pivot_epsilon_v,
)


class Pivot(Subsystem):
    def __init__(self, scheduler: CommandScheduler, elevator: Elevator):
        scheduler.registerSubsystem(self)

        self.elevator: Elevator = elevator

        self.pivot_motors = [
            SparkFlex(motor_id, SparkFlex.MotorType.kBrushless)
            for motor_id in pivot_motor_ids
        ]

        self.pivot_motor_encoders = [motor.getEncoder() for motor in self.pivot_motors]

        for i, motor in enumerate(self.pivot_motors):
            config = SparkBaseConfig()
            config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
            config.inverted(i % 2 == 1)  # Alternate inverting for correct rotation
            motor.configure(
                config,
                SparkFlex.ResetMode.kResetSafeParameters,
                SparkFlex.PersistMode.kNoPersistParameters,
            )

        self.theta_pid = ProfiledPIDController(
            *pivot_pid_constants,
            constraints=TrapezoidProfile.Constraints(*pivot_pid_constraint_constants),
        )

        self.pivot_encoder = DutyCycleEncoder(pivot_encoder_id)

        self.setpoint: float = 0

    def periodic(self):
        self.target_angle(self.setpoint)

    def target_angle(self, target):
        pid_output = self.theta_pid.calculate(self.get_angle(), target)
        self.set_power(pid_output + self.elevator.ff_power())

    def set_power(self, power: float):
        for motor in self.pivot_motors:
            motor.set(power)

    def get_angle(self) -> float:
        return self.pivot_encoder.get() * 2.0 * pi - pivot_angle_offset

    def at_angle(self) -> bool:
        if (
            abs(self.get_angle() - self.setpoint) < pivot_epsilon_pos
            and abs(self.pivot_motor_encoders[0].getVelocity()) < pivot_epsilon_v
        ):
            return True

        return False

    def target_attainable(self) -> bool:
        return True
