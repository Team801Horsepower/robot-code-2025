from commands2 import CommandScheduler, Subsystem
from wpilib import DutyCycleEncoder
from wpimath.controller import ProfiledPIDController
from math import pi
from wpimath.trajectory import TrapezoidProfile
from rev import SparkFlex, SparkBaseConfig
from navx import AHRS


from math import sin, cos

from subsystems.elevator import Elevator
from config import (
    pivot_motor_ids,
    pivot_pid_constants,
    pivot_pid_constraint_constants,
    pivot_encoder_id,
    pivot_angle_offset,
    pivot_epsilon_pos,
    pivot_epsilon_v,
    pivot_range,
    g,
)


class Pivot(Subsystem):
    def __init__(self, scheduler: CommandScheduler, elevator: Elevator, navx: AHRS):
        self.elevator: Elevator = elevator
        self.navx: AHRS = navx

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

        self.target_angle: float = 0  # TODO: Shouldn't default to 0
        self.current_angle = pivot_range[0]

        scheduler.registerSubsystem(self)

    def periodic(self):
        self.target_target_angle(self.target_angle)
        self.current_angle = self.update_angle()

    def target_target_angle(self, target: float):
        pid_output = self.theta_pid.calculate(self.get_angle(), target)
        self.set_power((pid_output * self.elevator.moi) + self.pivot_ff_torque())

    def set_power(self, power: float):
        for motor in self.pivot_motors:
            motor.set(power)

    def get_angle(self) -> float:
        return self.current_angle

    def update_angle(self) -> float:
        return self.pivot_encoder.get() * 2.0 * pi - pivot_angle_offset

    def at_angle(self) -> bool:
        if (
            abs(self.get_angle() - self.target_angle) < pivot_epsilon_pos
            and abs(self.pivot_motor_encoders[0].getVelocity()) < pivot_epsilon_v
        ):
            return True

        return False

    def target_attainable(self) -> bool:
        return (
            pivot_range[0] <= self.target_angle and self.target_angle <= pivot_range[1]
        )

    def pivot_ff_torque(self):
        t_g = self.elevator.r_com * self.elevator.mass * sin(self.get_angle()) * g
        t_a = (
            self.elevator.r_com
            * self.elevator.mass
            * cos(self.get_angle())
            * self.navx.getRawAccelX()
        )

        return t_g + t_a
