from commands2 import CommandScheduler, Subsystem
from rev import SparkFlex, SparkFlexConfig
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile

from config import (
    elevator_mass,
    elevator_dynamics_table,
    extension_range,
    extension_ratio,
    extension_motor_ids,
    extension_pid_constants,
    extension_pid_constraint_constants,
)
from utils import lerp_over_table
from subsystems.pivot import Pivot


class Elevator(Subsystem):
    def __init__(self, scheduler: CommandScheduler, pivot: Pivot):
        scheduler.registerSubsystem(self)

        self.pivot: Pivot = pivot

        self.mass = elevator_mass
        self.target_extension = 0.0

        self.extension_motors = [
            SparkFlex(motor_id, SparkFlex.MotorType.kBrushless)
            for motor_id in extension_motor_ids
        ]

        self.extension_motor_encoders = [
            motor.getEncoder() for motor in self.extension_motors
        ]

        for i, motor in enumerate(self.extension_motors):
            config = SparkFlexConfig()
            config.setIdleMode(SparkFlexConfig.IdleMode.kBrake)
            config.inverted(i % 2 == 1)  # Alternate inverting for correct rotation
            motor.configure(
                config,
                SparkFlex.ResetMode.kResetSafeParameters,
                SparkFlex.PersistMode.kNoPersistParameters,
            )

        self.extension_pid = ProfiledPIDController(
            *extension_pid_constants,
            constraints=TrapezoidProfile.Constraints(
                *extension_pid_constraint_constants
            ),
        )

        self.extension: float = extension_range[0]

    def periodic(self):
        pass

    def set_power(self, power: float):
        for motor in self.extension_motors:
            motor.set(power)

    def at_extension(self) -> bool:
        return False

    def target_attainable(self) -> bool:
        return (
            extension_range[0] <= self.target_extension
            and self.target_extension <= extension_range[1]
        )

    def update_extension(self) -> float:
        return (
            self.extension_motor_encoders[0].getPosition() * extension_ratio
            + extension_range[0]
        )

    def get_extension(self) -> float:
        return self.extension

    @property
    def r_com(self) -> float:
        return lerp_over_table(elevator_dynamics_table, self.get_extension())[0]

    @property
    def moi(self):
        return lerp_over_table(elevator_dynamics_table, self.get_extension())[1]
