from commands2 import CommandScheduler, Subsystem
from wpilib import DutyCycleEncoder, SmartDashboard
from wpimath.controller import ProfiledPIDController
from math import pi
from wpimath.trajectory import TrapezoidProfile
from wpimath import units
from rev import SparkFlex, SparkBaseConfig
from navx import AHRS


from math import sin, cos
import time

from utils import lerp_over_table, clamp, time_f
from subsystems.elevator import Elevator
import config


class Pivot(Subsystem):
    def __init__(self, scheduler: CommandScheduler, elevator: Elevator, navx: AHRS):
        scheduler.registerSubsystem(self)

        self.elevator: Elevator = elevator
        self.navx: AHRS = navx

        self.pivot_motors = [
            SparkFlex(motor_id, SparkFlex.MotorType.kBrushless)
            for motor_id in config.pivot_motor_ids
        ]

        self.pivot_motor_encoders = [motor.getEncoder() for motor in self.pivot_motors]

        for i, motor in enumerate(self.pivot_motors):
            motor_config = SparkBaseConfig()
            motor_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
            # Alternate inverting for correct rotation, positive is up
            motor_config.inverted(i % 2 == 0)
            motor_config.closedLoop.pid(0.01, 0, 0)
            motor.configure(
                motor_config,
                SparkFlex.ResetMode.kResetSafeParameters,
                SparkFlex.PersistMode.kNoPersistParameters,
            )

        self.pivot_motor_controllers = [
            motor.getClosedLoopController() for motor in self.pivot_motors
        ]

        self.theta_pid = ProfiledPIDController(
            *lerp_over_table(config.pivot_pid_constants, self.elevator.get_extension()),
            constraints=TrapezoidProfile.Constraints(
                *config.pivot_pid_constraint_constants
            ),
        )
        self.climb_theta_pid = ProfiledPIDController(
            config.pivot_climb_p,
            0,
            0,
            constraints=TrapezoidProfile.Constraints(
                1000, config.climbing_pivot_acc_limit
            ),
        )
        # This should probably be removed, but I won't be the one to do it.
        self.climb_theta_pid.setConstraints(
            TrapezoidProfile.Constraints(1000, config.climbing_pivot_acc_limit)
        )

        self.pivot_encoder = DutyCycleEncoder(config.pivot_encoder_id)

        self.target_angle: float = pi / 2
        self.current_angle = config.pivot_range[0]

        self.last_velocity = 0
        self.last_update_time = time.time()
        self.acceleration = 0

        self.has_flipped_middle_finger = False
        self.climbing = False

        self.last_direction = 0

        SmartDashboard.putNumber("climbing acc limit", config.climbing_pivot_acc_limit)

    @time_f("periodic pivot")
    def periodic(self):
        config.climbing_pivot_acc_limit = SmartDashboard.getNumber(
            "climbing acc limit", config.climbing_pivot_acc_limit
        )

        self.target_target_angle(self.target_angle)
        self.current_angle = self.update_angle()
        new_constants = lerp_over_table(
            config.pivot_pid_constants, self.elevator.get_extension()
        )
        self.theta_pid.setP(new_constants[0])
        self.theta_pid.setI(new_constants[1])
        self.theta_pid.setD(new_constants[2])

        self.theta_pid.setConstraints(
            TrapezoidProfile.Constraints(
                1000,
                lerp_over_table(config.pivot_acc_lim, self.elevator.get_extension())[0],
            )
        )

        self.acceleration = (
            self.pivot_motor_encoders[0].getVelocity() - self.last_velocity
        ) / (time.time() - self.last_update_time)
        self.last_velocity = self.pivot_motor_encoders[0].getVelocity()
        self.last_update_time = time.time()

        SmartDashboard.putNumber(
            "pivot angle", units.radiansToDegrees(self.get_angle())
        )
        SmartDashboard.putNumber(
            "pivot target", units.radiansToDegrees(self.target_angle)
        )
        SmartDashboard.putNumber(
            "pivot encoder", self.pivot_motor_encoders[0].getPosition()
        )

        SmartDashboard.putNumber(
            "pivot current", self.pivot_motors[0].getOutputCurrent()
        )

    def target_target_angle(self, target: float):
        self.has_flipped_middle_finger |= self.get_angle() >= config.middle_finger_angle
        if not self.has_flipped_middle_finger:
            target = config.middle_finger_angle + units.degreesToRadians(2)

        normal_pid_output = self.theta_pid.calculate(self.get_angle(), target)
        climb_pid_output = self.climb_theta_pid.calculate(self.get_angle(), target)

        if self.climbing:
            pid_output = climb_pid_output
            if self.get_angle() < config.climb_power_increase_angle:
                pid_output *= config.climb_power_mult_when_low
            else:
                pid_output *= config.climb_power_mult
            self.set_power(pid_output)
            return

        pid_output = normal_pid_output

        current_direction = 1 if pid_output > 0 else -1
        if current_direction != self.last_direction:
            backlash_compensator = lerp_over_table(
                config.pivot_backlash_compensator, self.elevator.get_extension()
            )[0]
            pid_output += current_direction * backlash_compensator
        self.last_direction = current_direction

        # self.set_power(pid_output + self.pivot_ff_power())
        self.set_current(pid_output + self.pivot_ff_power())

    def set_power(self, power: float):
        SmartDashboard.putNumber("pivot power", power)
        if not self.climbing:
            power = clamp(-0.25, 0.25, power)
        for motor in self.pivot_motors:
            motor.set(power)

    def set_current(self, current: float):
        SmartDashboard.putNumber("pivot target current", current)
        current = clamp(-40, 40, current)
        for controller in self.pivot_motor_controllers:
            controller.setReference(current, SparkFlex.ControlType.kCurrent)

    def get_angle(self) -> float:
        return self.current_angle

    def update_angle(self) -> float:
        return config.pivot_angle_offset - self.pivot_encoder.get() * 2.0 * pi

    def get_vel(self) -> float:
        return (
            self.pivot_motor_encoders[0].getVelocity()
            / 60
            / config.pivot_gear_ratio
            * 2
            * pi
        )

    def at_angle(self) -> bool:
        at_pos = abs(self.get_angle() - self.target_angle) < config.pivot_epsilon_pos
        at_vel = abs(self.get_vel()) < config.pivot_epsilon_v
        SmartDashboard.putBoolean("pivot pos at target", at_pos)
        SmartDashboard.putBoolean("pivot vel at target", at_vel)
        SmartDashboard.putNumber("pivot vel", self.get_vel())
        if at_pos and at_vel:
            return True

        return False

    def target_attainable(self) -> bool:
        return (
            config.pivot_range[0] <= self.target_angle
            and self.target_angle <= config.pivot_range[1]
        )

    def pivot_ff_power(self):
        t_g = (
            self.elevator.ff_scaler
            * cos(self.get_angle() - config.pivot_com_offset_for_feedforward)
            * config.g
        )
        t_a = (
            self.elevator.ff_scaler
            * self.elevator.mass
            * sin(self.get_angle() - config.pivot_com_offset_for_feedforward)
            * self.navx.getRawAccelX()
        )

        return t_g - t_a
