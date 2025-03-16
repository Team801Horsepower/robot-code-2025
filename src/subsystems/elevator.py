from commands2 import CommandScheduler, Subsystem
from rev import SparkFlex, SparkFlexConfig
from wpimath.controller import PIDController
from wpilib import SmartDashboard

import config
from utils import lerp_over_table, clamp, time_f


class Elevator(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        # self.pivot: Pivot = pivot

        self.mass = config.elevator_mass
        self.target_extension = config.extension_range[0]

        self.extension_motors = [
            SparkFlex(motor_id, SparkFlex.MotorType.kBrushless)
            for motor_id in config.extension_motor_ids
        ]

        self.extension_motor_encoders = [
            motor.getEncoder() for motor in self.extension_motors
        ]

        for i, motor in enumerate(self.extension_motors):
            motor_config = SparkFlexConfig()
            motor_config.setIdleMode(SparkFlexConfig.IdleMode.kCoast)
            # Alternate inverting for correct rotation
            motor_config.inverted(i % 2 == 1)
            motor.configure(
                motor_config,
                SparkFlex.ResetMode.kResetSafeParameters,
                SparkFlex.PersistMode.kNoPersistParameters,
            )

        # self.extension_pid = ProfiledPIDController(
        #     *config.extension_pid_constants,
        #     constraints=TrapezoidProfile.Constraints(
        #         *extension_pid_constraint_constants
        #     ),
        # )

        self.extension_pid = PIDController(*config.extension_pid_constants)

        self.extension: float = config.extension_range[0]
        self.extension_vel: float = 0

        # Whether the wrist position is high enough to allow the elevator to lower
        self.wrist_up = True

    @time_f("periodic elevator")
    def periodic(self):
        target = self.target_extension
        if not self.wrist_up:
            target = max(target, config.wrist_passthrough_min_extension)
        self.target_target_extension(target)
        self.update_extension()

        SmartDashboard.putNumber("elevator extension", self.get_extension())
        SmartDashboard.putNumber("elevator target", self.target_extension)

    def target_target_extension(self, target):
        target = clamp(
            config.extension_range[0] + 0.01, config.extension_range[1], target
        )
        pid_output = self.extension_pid.calculate(self.extension, target)

        resist_component = (
            -self.resist_coef * self.extension_vel * abs(self.extension_vel)
        )

        power = pid_output + config.elevator_ff_power + resist_component

        self.set_power(power)

        SmartDashboard.putNumber("elevator power", power)

    def set_power(self, power: float):
        for motor in self.extension_motors:
            motor.set(power)

    def at_extension(self) -> bool:
        return abs(self.get_extension() - self.target_extension) < 0.02

    def target_attainable(self) -> bool:
        return (
            config.extension_range[0] <= self.target_extension
            and self.target_extension <= config.extension_range[1]
        )

    def update_extension(self):
        self.extension = (
            self.extension_motor_encoders[0].getPosition() * config.extension_ratio
            + config.extension_range[0]
        )

        # Why do encoders output velocity in RPM :(
        self.extension_vel = (
            self.extension_motor_encoders[0].getVelocity() / 60 * config.extension_ratio
        )

    def get_extension(self) -> float:
        return self.extension

    @property
    def ff_scaler(self) -> float:
        return lerp_over_table(config.elevator_dynamics_table, self.extension)[0]

    @property
    def moi(self):
        return lerp_over_table(config.elevator_dynamics_table, self.extension)[1]

    @property
    def resist_coef(self) -> float:
        return 0.08
