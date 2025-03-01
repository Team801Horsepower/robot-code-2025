from commands2 import CommandScheduler, Subsystem
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d, Transform2d, Translation2d
from wpimath.units import radiansToDegrees, degreesToRadians
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
from utils import clamp


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
        if isinstance(self.pivot_relative_target, Transform2d):
            self._target_outofbounds = False
            # Position of the wrist pivot in 2D arm space
            wrist_position = self.pivot_relative_target.translation() - Translation2d(
                claw_to_wrist_lengths["algae" if self.use_algae else "coral"],
                Rotation2d(self.pivot_relative_target.rotation().radians()
                + self.use_algae * coral_algae_pickup_angle),
            )
            target_to_p_1_norm = self.pivot_relative_target.translation() - wrist_position
            target_to_p_1_norm /= target_to_p_1_norm.norm()
            claw_bounds = (
                Translation2d(*target_to_p_1_norm).rotateBy(Rotation2d(pi / 2))
                * claw_up_down_lengths[0]
                + self.pivot_relative_target.translation(),
                Translation2d(*target_to_p_1_norm).rotateBy(Rotation2d(pi / 2))
                * claw_up_down_lengths[1]
                + self.pivot_relative_target.translation(),
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
                wrist_position = Translation2d(
                    wrist_position.x
                    - (
                        max_point.x
                        + pivot_offset.x
                        - robot_dimensions.x / 2
                        - bumper_distance
                        - ik_boundary_distance
                    ),
                    wrist_position.y,
                )
                self._target_outofbounds = True
            elif min_point.x + pivot_offset.x < (
                -robot_dimensions.x / 2 - bumper_distance - ik_boundary_distance
            ):
                wrist_position = Translation2d(
                    wrist_position.x
                    + (
                        min_point.x
                        + pivot_offset.x
                        + robot_dimensions.x / 2
                        + bumper_distance
                        + ik_boundary_distance
                    ),
                    wrist_position.y,
                )
                self._target_outofbounds = True
            if min_point.y + pivot_offset.y < ik_floor:
                wrist_position = Translation2d(
                    wrist_position.x, wrist_position.y - (min_point.y + pivot_offset.y - ik_floor)
                )
                self._target_outofbounds = True

            # self.pivot.target_angle = max(min(p_1.angle().radians(), 50), 110)
            SmartDashboard.putNumber(
                "IK pivot angle",
                radiansToDegrees(
                    # clamp(
                    #     degreesToRadians(50),
                    #     degreesToRadians(100),
                        wrist_position.angle().radians(),
                    # )
                ),
            )
            # self.wrist.target_angle = pi - (
            #     p_1.angle().radians()
            #     - self.pivot_relative_target.rotation().radians()
            #     + float(self.use_algae) * coral_algae_pickup_angle
            # )
            SmartDashboard.putNumber(
                "IK wrist angle",
                radiansToDegrees(
                    pi
                    - (
                        wrist_position.angle().radians()
                        - self.pivot_relative_target.rotation().radians()
                        + float(self.use_algae) * coral_algae_pickup_angle
                    )
                ),
            )
            if self.should_extend:
                # self.elevator.target_extension = p_1.norm()
                SmartDashboard.putNumber("IK elevator extension", wrist_position.norm())
            else:
                # self.elevator.target_extension = extension_range[0]
                SmartDashboard.putNumber("IK elevator extension", extension_range[0])
        else:
            match self.pivot_relative_target:
                case (float(_), float(_), float(_)):
                    self.pivot.target_angle = self.pivot_relative_target[0]
                    self.elevator.target_extension = self.pivot_relative_target[1]
                    self.wrist.target_angle = self.pivot_relative_target[2]
                case _:
                    pass

    def at_target(self) -> bool:
        return (
            self.pivot.at_angle()
            and self.elevator.at_extension()
            and self.wrist.at_angle()
        )

    # TODO: Target is not necessarily a transform2d
    @property
    def pivot_relative_target(self) -> Optional[Transform2d | Tuple[float, float, float]]:
        match self.target:
            case None:
                return None
            case Transform2d():
                return Transform2d(self.target.x - pivot_offset.x, self.target.y - pivot_offset.y, self.target.rotation().radians())
            case (float(_), float(_), float(_)):
                return self.target
            case _:
                return

    def target_attainable(self) -> bool:
        return (
            self._target_outofbounds
            and self.pivot.target_attainable()
            and self.elevator.target_attainable()
            and self.wrist.target_attainable()
        )
