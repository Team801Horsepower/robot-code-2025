from commands2 import CommandScheduler, Subsystem
from wpilib import DutyCycleEncoder, SmartDashboard
from wpimath.controller import ProfiledPIDController, PIDController
from math import pi
from wpimath.trajectory import TrapezoidProfile
from rev import SparkFlex, SparkBaseConfig
from navx import AHRS


from math import sin, cos

from utils import lerp_over_table
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
    pivot_com_offset_for_feedforward,
    pivot_acc_lim,
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
            config.inverted(
                i % 2 == 0
            )  # Alternate inverting for correct rotation, positive is up
            motor.configure(
                config,
                SparkFlex.ResetMode.kResetSafeParameters,
                SparkFlex.PersistMode.kNoPersistParameters,
            )

        self.theta_pid = ProfiledPIDController(
            *lerp_over_table(pivot_pid_constants, self.elevator.get_extension()),
            constraints=TrapezoidProfile.Constraints(*pivot_pid_constraint_constants),
        )

        # self.theta_pid = PIDController(*pivot_pid_constants)

        self.pivot_encoder = DutyCycleEncoder(pivot_encoder_id)

        self.target_angle: float = pi / 2  # TODO: Shouldn't default to 0
        self.current_angle = pivot_range[0]

        scheduler.registerSubsystem(self)

    def periodic(self):
        self.target_target_angle(self.target_angle)
        self.current_angle = self.update_angle()
        new_constants = lerp_over_table(
            pivot_pid_constants, self.elevator.get_extension()
        )
        self.theta_pid.setP(new_constants[0])
        self.theta_pid.setI(new_constants[1])
        self.theta_pid.setD(new_constants[2])
        SmartDashboard.putNumber("kP", new_constants[0])
        SmartDashboard.putNumber("kI", new_constants[1])
        SmartDashboard.putNumber("kD", new_constants[2])
        self.theta_pid.setConstraints(
            TrapezoidProfile.Constraints(
                1000, lerp_over_table(pivot_acc_lim, self.elevator.get_extension())[0]
            )
        )
        SmartDashboard.putNumber(
            "pivot acc limit",
            lerp_over_table(pivot_acc_lim, self.elevator.get_extension())[0],
        )
        SmartDashboard.putNumber("ff torque", self.pivot_ff_power())

    def target_target_angle(self, target: float):
        pid_output = self.theta_pid.calculate(self.get_angle(), target)
        # pid_output = pid_output * (1 + (self.elevator.get_extension() * 2 / 65))
        self.set_power(pid_output + self.pivot_ff_power())
        # self.set_power(self.pivot_ff_torque())

    def set_power(self, power: float):
        power = min(0.1, max(-0.1, power))
        SmartDashboard.putNumber("pvt pwr", power)
        for motor in self.pivot_motors:
            motor.set(power)

    def get_angle(self) -> float:
        return self.current_angle

    def update_angle(self) -> float:
        return pivot_angle_offset - self.pivot_encoder.get() * 2.0 * pi

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

    def pivot_ff_power(self):
        t_g = (
            # self.elevator.get_extension()
            self.elevator.ff_scaler
            * cos(self.get_angle() - pivot_com_offset_for_feedforward)
            * g
        )
        t_a = (
            self.elevator.ff_scaler
            * self.elevator.mass
            * sin(self.get_angle() - pivot_com_offset_for_feedforward)
            * self.navx.getRawAccelX()
        )

        return t_g + t_a
