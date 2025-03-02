from commands2 import CommandScheduler, Subsystem
from rev import SparkFlex, SparkFlexConfig
from wpimath.controller import PIDController

from config import (
    elevator_mass,
    elevator_dynamics_table,
    extension_range,
    extension_ratio,
    extension_motor_ids,
    extension_pid_constants,
    elevator_ff_power,
    wrist_passthrough_min_extension,
)
from utils import lerp_over_table, clamp


class Elevator(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        # self.pivot: Pivot = pivot

        self.mass = elevator_mass
        self.target_extension = extension_range[0]

        self.extension_motors = [
            SparkFlex(motor_id, SparkFlex.MotorType.kBrushless)
            for motor_id in extension_motor_ids
        ]

        self.extension_motor_encoders = [
            motor.getEncoder() for motor in self.extension_motors
        ]

        for i, motor in enumerate(self.extension_motors):
            config = SparkFlexConfig()
            config.setIdleMode(SparkFlexConfig.IdleMode.kCoast)
            config.inverted(i % 2 == 1)  # Alternate inverting for correct rotation
            motor.configure(
                config,
                SparkFlex.ResetMode.kResetSafeParameters,
                SparkFlex.PersistMode.kNoPersistParameters,
            )

        # self.extension_pid = ProfiledPIDController(
        #     *extension_pid_constants,
        #     constraints=TrapezoidProfile.Constraints(
        #         *extension_pid_constraint_constants
        #     ),
        # )

        self.extension_pid = PIDController(*extension_pid_constants)

        self.extension: float = extension_range[0]

        # Whether the wrist position is high enough to allow the elevator to lower
        self.wrist_up = True

    def periodic(self):
        target = self.target_extension
        if not self.wrist_up:
            target = max(target, wrist_passthrough_min_extension)
        self.target_target_extension(target)
        self.extension = self.update_extension()

    def target_target_extension(self, target):
        target = clamp(extension_range[0] + 0.01, extension_range[1], target)
        pid_output = self.extension_pid.calculate(self.extension, target)
        self.set_power(pid_output + elevator_ff_power)

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
    def ff_scaler(self) -> float:
        return lerp_over_table(elevator_dynamics_table, self.extension)[0]

    @property
    def moi(self):
        return lerp_over_table(elevator_dynamics_table, self.extension)[1]
