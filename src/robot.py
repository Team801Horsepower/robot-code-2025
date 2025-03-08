#!/usr/bin/env python3
from math import atan2, floor, pi, sqrt, pow

import wpilib

from wpilib import SmartDashboard

from wpimath.geometry import Transform2d, Pose2d, Rotation2d, Translation2d
from commands2 import CommandScheduler
from wpimath import units
import time

import config
from subsystems import drive, periscope, vision
from commands.drive_to_pose import DriveToPose
from commands.graph_pathfind import GraphPathfind
from commands.target_reef import TargetReef


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.scheduler = CommandScheduler()

        # Subsystem initialization
        self.scheduler.unregisterAllSubsystems()

        self.drive = drive.Drive(self.scheduler)

        self.periscope = periscope.Periscope(self.scheduler, self.drive.odometry.ahrs)

        self.vision = vision.Vision(self.scheduler)

        self.driver_controller = wpilib.XboxController(0)
        self.manip_controller = wpilib.XboxController(1)

        self.drive.chassis.set_swerves()

        self.field_oriented = True

        self.manip_setpoint = config.reef_setpoints[1]

        self.last_loop = time.time()

        for encoder in self.periscope.arm.elevator.extension_motor_encoders:
            encoder.setPosition(0)

        SmartDashboard.putNumber("approach P", 15.0)

    def robotPeriodic(self):
        # This line must always be present in robotPeriodic, or else
        # commands and subsystem periodic methods will not run
        self.scheduler.run()

        SmartDashboard.putNumber("pivot angle", self.periscope.arm.pivot.get_angle())
        SmartDashboard.putNumber(
            "elevator extension", self.periscope.arm.elevator.get_extension()
        )
        SmartDashboard.putNumber(
            "wrist pos",
            units.radiansToDegrees(self.periscope.arm.wrist.angle()),
        )

        SmartDashboard.putNumber("heading", self.drive.odometry.rotation().degrees())

        heading = self.drive.odometry.rotation().radians()
        # ntags, estimates = self.vision.estimate_multitag_pos(heading)
        # for i, ((id1, id2), (pos, conf)) in enumerate(estimates):
        # SmartDashboard.putNumber(f"x {i}", units.metersToInches(pos.x))
        # SmartDashboard.putNumber(f"y {i}", units.metersToInches(pos.y))
        # SmartDashboard.putNumber(f"confidence {i}", conf)
        # SmartDashboard.putString(f"pair {i}", f"{id1}–{id2}")

        # ntags = pos = conf = dev = heading_correction = None
        # pos = conf = dev = heading_correction = None

        n_ests, report = self.vision.pos_report(heading)
        if report is not None:
            pos, conf, dev = report
            SmartDashboard.putNumber("avg x", units.metersToInches(pos.x))
            SmartDashboard.putNumber("avg y", units.metersToInches(pos.y))
            SmartDashboard.putNumber("composite conf", conf)
            SmartDashboard.putNumber("deviation", dev)

            SmartDashboard.putNumberArray("vision pose", [pos.x, pos.y, heading])

            # heading_correction = self.vision.heading_correction(heading)
            # if heading_correction is not None:
            #     SmartDashboard.putNumber(
            #         "corrected heading", units.radiansToDegrees(heading_correction)
            #     )
            # speeds = self.drive.chassis.chassis_speeds()
            # speed = sqrt(speeds.vx**2 + speeds.vy**2)
            # if conf >= 0.25 and speed < 0.15:
            #     # if conf >= 0.2:
            if n_ests > 1 and conf >= 0.35 and dev < 0.1:
                # if heading_correction is not None and abs(
                #     heading_correction - heading
                # ) < units.degreesToRadians(5):
                #     # self.drive.odometry.reset_heading(Rotation2d(heading_correction))
                #     pass
                self.drive.odometry.reset_translation(pos)

        robot_pose = self.drive.odometry.pose()
        robot_pos = robot_pose.translation()
        SmartDashboard.putNumber("robot x", robot_pos.x)
        SmartDashboard.putNumber("robot y", robot_pos.y)

        SmartDashboard.putNumberArray("robot pose", [robot_pos.x, robot_pos.y, heading])

        now = time.time()
        took = now - self.last_loop
        self.last_loop = now
        SmartDashboard.putNumber("loop time (cs)", took * 100)

        tag8_pose = self.vision.layout.getTagPose(8)
        if tag8_pose is not None:
            tag8_pos = tag8_pose.toPose2d().translation()
            fr_swerve_pos = robot_pos + Translation2d(
                config.robot_dimensions.x, -config.robot_dimensions.y
            ).rotateBy(robot_pose.rotation())
            dist = (fr_swerve_pos - tag8_pos).norm()
            SmartDashboard.putNumber("tag8-FR swerve (in)", units.metersToInches(dist))

        def get_yaw(i: int) -> float | None:
            targets = self.vision.results[i].getTargets()
            for target in targets:
                if target.getFiducialId() == 8:
                    return units.degreesToRadians(target.getYaw())
            return None

        left_yaw = get_yaw(0)
        right_yaw = get_yaw(2)

        if left_yaw is not None and right_yaw is not None:
            SmartDashboard.putNumber(
                "rp diff", units.radiansToDegrees(left_yaw - right_yaw)
            )

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass

    def disabledExit(self):
        pass

    def autonomousInit(self):
        # dtp = DriveToPose(Pose2d(14.108, 6.0818, Rotation2d()), self.drive)
        # dtp = DriveToPose(
        #     Pose2d(16.485, 7.164, Rotation2d.fromDegrees(230)), self.drive
        # )
        # pose = self.drive.odometry.pose().transformBy(
        #     Transform2d(units.inchesToMeters(-60), 0, 0)
        # )
        # dtp = DriveToPose(pose, self.drive)
        # self.scheduler.schedule(dtp)
        # self.periscope.arm.target = config.source_setpoint

        self.periscope.arm.target = config.reef_setpoints[1]
        tr = TargetReef(self.drive, self.vision, 9)
        self.scheduler.schedule(tr)

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        pass

    def teleopInit(self):
        self.periscope.arm.target = Transform2d(
            config.ik_neutral_x, config.ik_neutral_y, config.ik_neutral_wrist
        )

        self.reef_selection = 0

        SmartDashboard.putNumber("new IK x", config.ik_neutral_x)
        SmartDashboard.putNumber("new IK y", config.ik_neutral_y)
        SmartDashboard.putNumber(
            "new IK wrist", units.radiansToDegrees(config.ik_neutral_wrist)
        )

        SmartDashboard.putNumber(
            "new pivot target",
            units.radiansToDegrees(self.periscope.arm.pivot.target_angle),
        )
        SmartDashboard.putNumber(
            "new elevator target", self.periscope.arm.elevator.target_extension
        )
        SmartDashboard.putNumber(
            "new wrist target",
            units.radiansToDegrees(self.periscope.arm.wrist.target_angle),
        )

    def teleopPeriodic(self):
        def deadzone(activation: float) -> float:
            if abs(activation) < 0.01:
                return 0.0
            return activation

        # Driving
        self.field_oriented ^= self.driver_controller.getBackButtonPressed()
        drive_input = Transform2d(
            deadzone(-self.driver_controller.getLeftY()) * config.drive_speed,
            deadzone(-self.driver_controller.getLeftX()) * config.drive_speed,
            deadzone(-self.driver_controller.getRightX()) * config.turn_speed,
        )
        self.drive.drive(drive_input, self.field_oriented)

        if self.driver_controller.getYButtonPressed():
            self.drive.chassis.zero_swerves()

        if self.driver_controller.getStartButtonPressed():
            self.drive.odometry.reset()

        # self.periscope.arm.should_extend = self.driver_controller.getRightBumper()

        if self.manip_controller.getAButtonPressed():
            self.manip_setpoint = config.reef_setpoints[0]
        elif self.manip_controller.getBButtonPressed():
            self.manip_setpoint = config.reef_setpoints[1]
        elif self.manip_controller.getXButtonPressed():
            self.manip_setpoint = config.reef_setpoints[2]
        elif self.manip_controller.getYButtonPressed():
            self.manip_setpoint = config.reef_setpoints[3]
        elif self.manip_controller.getStartButtonPressed():
            self.manip_setpoint = config.processor_setpoint
        elif self.manip_controller.getBackButtonPressed():
            self.manip_setpoint = config.barge_setpoint

        if self.driver_controller.getRightBumper():
            if (
                self.driver_controller.getXButton()
                or self.driver_controller.getBButton()
            ):
                setpoint = config.source_setpoint
            else:
                setpoint = self.manip_setpoint
            self.periscope.arm.target = setpoint
            self.periscope.arm.should_extend = True
        elif self.driver_controller.getLeftBumper():
            self.periscope.arm.target = config.algae_reef_setpoints[1]
            self.periscope.arm.should_extend = True
        else:
            self.periscope.arm.target = config.transit_setpoint
            self.periscope.arm.should_extend = False

        # if self.driver_controller.getAButtonPressed():
        #     self.read_typed_ik_input()
        # self.periscope.arm.should_extend ^= (
        #     self.driver_controller.getRightBumperPressed()
        # )

        claw_power = (
            self.driver_controller.getLeftTriggerAxis()
            - self.driver_controller.getRightTriggerAxis()
        )
        self.periscope.claw.set(claw_power)
        if (
            abs(self.manip_controller.getLeftX()) > 0.5
            or abs(self.manip_controller.getLeftY()) > 0.5
        ):
            fullcircle = lambda x: (2 * pi - abs(x)) if x < 0 else x
            self.reef_selection = floor(
                6
                * fullcircle(
                    atan2(
                        -self.manip_controller.getLeftY(),
                        self.manip_controller.getLeftX(),
                    )
                    + pi / 24
                )
                / pi
            )
        SmartDashboard.putNumber("reef selection", self.reef_selection)

    def teleopExit(self):
        pass

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def testExit(self):
        pass

    def read_typed_ik_input(self):
        new_ik_x = SmartDashboard.getNumber("new IK x", config.ik_neutral_x)
        new_ik_y = SmartDashboard.getNumber("new IK y", config.ik_neutral_y)
        new_ik_wrist = units.degreesToRadians(
            SmartDashboard.getNumber(
                "new IK wrist", units.radiansToDegrees(config.ik_neutral_wrist)
            )
        )
        self.periscope.arm.target = Transform2d(new_ik_x, new_ik_y, new_ik_wrist)

    def read_typed_arm_input(self):
        new_pivot_target = units.degreesToRadians(
            SmartDashboard.getNumber(
                "new pivot target",
                units.radiansToDegrees(self.periscope.arm.pivot.target_angle),
            )
        )
        new_extension_target = SmartDashboard.getNumber(
            "new elevator target", self.periscope.arm.pivot.target_angle
        )
        new_wrist_target = units.degreesToRadians(
            SmartDashboard.getNumber(
                "new wrist target",
                units.radiansToDegrees(self.periscope.arm.wrist.target_angle),
            )
        )
        self.periscope.arm.target = (
            new_pivot_target,
            new_extension_target,
            new_wrist_target,
        )


if __name__ == "__main__":
    wpilib.run(Robot)
