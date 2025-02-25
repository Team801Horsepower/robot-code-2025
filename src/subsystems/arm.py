from commands2 import CommandScheduler, Subsystem
from wpimath.geometry import Transform2d, Translation2d
from typing import Optional
from math import pi

from subsystems.pivot import Pivot
from subsystems.elevator import Elevator
from subsystems.wrist import Wrist

from config import claw_to_wrist_lengths, coral_algae_pickup_angle


class Arm(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.pivot = Pivot(scheduler)
        self.elevator = Elevator(scheduler)
        self.wrist = Wrist(scheduler)

        self.target: Optional[Transform2d] = None
        self.use_algae = 0
        self.should_extend = False

    def periodic(self):
        if self.target is not None:
            # Position of the wrist pivot in 2D arm space
            p_1 = Translation2d(
                claw_to_wrist_lengths["algae" if self.use_algae else "coral"], 0
            ).rotate(self.target.rotation() + self.use_algae * coral_algae_pickup_angle)

            self.pivot.target_angle = p_1.angle()
            # TODO: Software limits on wrist
            self.wrist.target_angle = pi - (
                p_1.angle()
                - self.target.rotation()
                + self.use_algae * coral_algae_pickup_angle
            )
            if self.should_extend:
                self.elevator.target_extension = p_1.norm()
            else:
                # TODO: Insert retracted elevator length
                self.elevator.target_extension = 0

    def at_target(self) -> bool:
        return (
            self.pivot.at_angle()
            and self.elevator.at_extension()
            and self.wrist.at_angle()
        )
