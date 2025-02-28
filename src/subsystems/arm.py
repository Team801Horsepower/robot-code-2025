from commands2 import CommandScheduler, Subsystem
from wpimath.geometry import Transform2d, Translation2d
from typing import Optional, Tuple
from math import pi

from subsystems.pivot import Pivot
from subsystems.elevator import Elevator
from subsystems.wrist import Wrist

from config import (
    claw_to_wrist_lengths,
    coral_algae_pickup_angle,
    claw_up_down_lengths,
    bumper_distance,
    pivot_offset,
    robot_dimensions,
    ik_boundary_distance,
    ik_floor,
    extension_range,
)


class Arm(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.pivot = Pivot(scheduler)
        self.elevator = Elevator(scheduler)
        self.wrist = Wrist(scheduler)

        self.target: Optional[Transform2d | Tuple[float, float, float]] = None
        self.use_algae = False
        self.should_extend = False
        self._target_outofbounds = False

        scheduler.registerSubsystem(self)

    def periodic(self):
        if isinstance(self.target, Transform2d):
            self._target_outofbounds = False
            # Position of the wrist pivot in 2D arm space
            p_1 = Translation2d(
                claw_to_wrist_lengths["algae" if self.use_algae else "coral"],
                self.target.rotation() + self.use_algae * coral_algae_pickup_angle,
            )
            claw_bounds = (
                Translation2d(target.translation() - p_1).norm().rotate(pi / 2)
                * claw_up_down_lengths[0]
                + target.translation(),
                Translation2d(-target.translation() - p_1).norm().rotate(pi / 2)
                * claw_up_down_lengths[1]
                + target.translation(),
            )
            max_point = (
                max(claw_bounds[0].x, claw_bounds[1].x),
                max(claw_bounds[0].y, claw_bounds[1].y),
            )
            min_point = (
                min(claw_bounds[0].x, claw_bounds[1].x),
                min(claw_bounds[0].y, claw_bounds[1].y),
            )
            if max_point.x + pivot_offset.x > (
                robot_dimensions.x / 2 + bumper_distance + ik_boundary_distance
            ):
                p_1.x -= (
                    max_point.x
                    + pivot_offset.x
                    - robot_dimensions.x / 2
                    - bumper_distance
                    - ik_boundary_distance
                )
                self._target_outofbounds = True
            elif min_point.x + pivot_offset.x < (
                -robot_dimensions.x / 2 - bumper_distance - ik_boundary_distance
            ):
                p_1.x += (
                    min_point.x
                    + pivot_offset.x
                    + robot_dimensions.x / 2
                    + bumper_distance
                    + ik_boundary_distance
                )
                self._target_outofbounds = True
            if min_point.y + pivot_offset.y < ik_floor:
                p_1.y -= min_point.y + pivot_offset.y - ik_floor
                self._target_outofbounds = True

            self.pivot.target_angle = max(min(p_1.angle(), 50), 110)
            self.wrist.target_angle = pi - (
                p_1.angle()
                - self.target.rotation()
                + float(self.use_algae) * coral_algae_pickup_angle
            )
            if self.should_extend:
                self.elevator.target_extension = p_1.norm()
            else:
                # TODO: Insert retracted elevator length
                self.elevator.target_extension = extension_range[0]
        elif isinstance(self.target, Tuple[float, float, float]):
            self.pivot.target = self.target[0]
            self.elevator.target = self.target[1]
            self.wrist.target_angle = self.target[2]

    def at_target(self) -> bool:
        return (
            self.pivot.at_angle()
            and self.elevator.at_extension()
            and self.wrist.at_angle()
        )

    def target_attainable(self) -> bool:
        return (
            self._target_outofbounds
            and self.pivot.target_attainable()
            and self.elevator.target_attainable()
            and self.wrist.target_attainable()
        )
