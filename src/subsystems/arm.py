from commands2 import CommandScheduler, Subsystem
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d, Transform2d, Translation2d
from wpimath.units import radiansToDegrees
from typing import Optional, Tuple
from math import pi
from navx import AHRS

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
    def __init__(self, scheduler: CommandScheduler, navx: AHRS):
        scheduler.registerSubsystem(self)

        self.elevator = Elevator(scheduler)
        self.pivot = Pivot(scheduler, self.elevator, navx)
        self.wrist = Wrist(scheduler)

        self.target: Optional[Transform2d | Tuple[float, float, float]] = None
        self.use_algae = False
        self.should_extend = False
        self._target_outofbounds = False
        # SmartDashboard.putNumber("Pivot Angle", 0)
        # SmartDashboard.putNumber("Elevator Extension", 0)
        # SmartDashboard.putNumber("Wrist Angle", 0)
        scheduler.registerSubsystem(self)

    def periodic(self):
        if isinstance(self.target, Transform2d):
            self._target_outofbounds = False
            # Position of the wrist pivot in 2D arm space
            p_1 = Translation2d(
                claw_to_wrist_lengths["algae" if self.use_algae else "coral"],
                self.target.rotation().radians()
                + self.use_algae * coral_algae_pickup_angle,
            )
            target_to_p_1_norm = self.target.translation() - p_1
            target_to_p_1_norm /= target_to_p_1_norm.norm()
            claw_bounds = (
                Translation2d(*target_to_p_1_norm).rotateBy(Rotation2d(pi / 2))
                * claw_up_down_lengths[0]
                + self.target.translation(),
                Translation2d(*target_to_p_1_norm).rotateBy(Rotation2d(pi / 2))
                * claw_up_down_lengths[1]
                + self.target.translation(),
            )
            max_point = Translation2d(
                max(claw_bounds[0].x, claw_bounds[1].x),
                max(claw_bounds[0].y, claw_bounds[1].y),
            )
            min_point = Translation2d(
                min(claw_bounds[0].x, claw_bounds[1].x),
                min(claw_bounds[0].y, claw_bounds[1].y),
            )
            if max_point.x + pivot_offset.x > (
                robot_dimensions.x / 2 + bumper_distance + ik_boundary_distance
            ):
                p_1 = Translation2d(
                    p_1.x
                    - (
                        max_point.x
                        + pivot_offset.x
                        - robot_dimensions.x / 2
                        - bumper_distance
                        - ik_boundary_distance
                    ),
                    p_1.y,
                )
                self._target_outofbounds = True
            elif min_point.x + pivot_offset.x < (
                -robot_dimensions.x / 2 - bumper_distance - ik_boundary_distance
            ):
                p_1 = Translation2d(
                    p_1.x
                    + (
                        min_point.x
                        + pivot_offset.x
                        + robot_dimensions.x / 2
                        + bumper_distance
                        + ik_boundary_distance
                    ),
                    p_1.y,
                )
                self._target_outofbounds = True
            if min_point.y + pivot_offset.y < ik_floor:
                p_1 = Translation2d(
                    p_1.x, p_1.y - (min_point.y + pivot_offset.y - ik_floor)
                )
                self._target_outofbounds = True

            # self.pivot.target_angle = max(min(p_1.angle().radians(), 50), 110)
            SmartDashboard.putNumber(
                "IK pivot angle",
                radiansToDegrees(max(min(p_1.angle().radians(), 50), 110)),
            )
            # self.wrist.target_angle = pi - (
            #     p_1.angle().radians()
            #     - self.target.rotation().radians()
            #     + float(self.use_algae) * coral_algae_pickup_angle
            # )
            SmartDashboard.putNumber(
                "IK wrist angle",
                radiansToDegrees(
                    pi
                    - (
                        p_1.angle().radians()
                        - self.target.rotation().radians()
                        + float(self.use_algae) * coral_algae_pickup_angle
                    )
                ),
            )
            if self.should_extend:
                # self.elevator.target_extension = p_1.norm()
                SmartDashboard.putNumber("IK elevator extension", p_1.norm())
            else:
                # self.elevator.target_extension = extension_range[0]
                SmartDashboard.putNumber("IK elevator extension", extension_range[0])
        else:
            match self.target:
                case (float(_), float(_), float(_)):
                    self.pivot.target_angle = self.target[0]
                    self.elevator.target_extension = self.target[1]
                    self.wrist.target_angle = self.target[2]
                case _:
                    pass

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
