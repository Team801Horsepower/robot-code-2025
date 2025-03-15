#!/usr/bin/env python3
from math import sin, atan2, floor, pi

import wpilib

from wpilib import SmartDashboard

from wpimath.geometry import Transform2d, Pose2d, Rotation2d, Translation2d
from commands2 import (
    CommandScheduler,
    FunctionalCommand,
    InstantCommand,
    WaitCommand,
    Command,
)
from wpimath import units
from functools import reduce
import time

import config
from subsystems import drive, odometry, periscope, vision, manipulator_controller

from commands.drive_to_pose import DriveToPose
from commands.graph_pathfind import GraphPathfind
from commands.target_reef import TargetReef
from commands.target_hps import TargetHPS
from commands.place_coral import PlaceCoral
from commands.collect_coral import CollectCoral
from commands.approach_reef import ApproachReef
from commands.approach_hps import ApproachHPS

from utils.graph import Graph
from utils import time_f

from auto_actions import make_auto_methods


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.scheduler = CommandScheduler()

        # Subsystem initialization
        self.scheduler.unregisterAllSubsystems()

        self.drive = drive.Drive(self.scheduler)

        self.periscope = periscope.Periscope(self.scheduler, self.drive.odometry.ahrs)

        self.vision = vision.Vision(self.scheduler)

        self.driver_controller = wpilib.XboxController(0)
        self.manip_controller = manipulator_controller.ManipulatorController(
            self.scheduler, 1
        )

        self.drive.chassis.set_swerves()

        self.field_oriented = True

        self.manip_setpoint = config.transit_setpoint

        self.last_loop = time.time()

        # Change variables = -1 if we left the zone, 1 if we entered
        self.last_near_reef = False
        self.near_reef_change = 0
        self.last_near_source = False
        self.near_source_change = 0

        for encoder in self.periscope.arm.elevator.extension_motor_encoders:
            encoder.setPosition(0)

        self.drive.odometry.reset(
            Pose2d(Translation2d(), Rotation2d(pi if config.is_red() else 0))
        )

        # SmartDashboard.putNumber("approach P", 15.0)

        SmartDashboard.putNumber("auto drive speed", config.auto_drive_speed)
        SmartDashboard.putNumber("auto turn speed", config.auto_turn_speed)

    def robotPeriodic(self):
        # This line must always be present in robotPeriodic, or else
        # commands and subsystem periodic methods will not run
        self.scheduler.run()

        config.auto_drive_speed = SmartDashboard.getNumber(
            "auto drive speed", config.auto_drive_speed
        )
        config.auto_turn_speed = SmartDashboard.getNumber(
            "auto turn speed", config.auto_turn_speed
        )

        self.drive.elevator_height = self.periscope.arm.elevator.get_extension() * sin(
            self.periscope.arm.pivot.get_angle()
        )
        self.drive.pivot_acceleration = self.periscope.arm.pivot.acceleration
        self.near_reef_change = self.drive.odometry.near_reef() - self.last_near_reef
        self.last_near_reef = self.drive.odometry.near_reef()
        self.near_source_change = (
            self.drive.odometry.near_source() - self.last_near_source
        )
        self.last_near_source = self.drive.odometry.near_source()

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
            if n_ests > 1 and conf >= 0.4 and dev < 0.05:
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
        SmartDashboard.putNumber("loop time", took)

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
        graph_path = config.code_path + "graph.json"
        graph = Graph(graph_path)

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

        # self.periscope.arm.target = config.reef_setpoints[1]
        self.periscope.arm.target = config.transit_setpoint
        # tr = TargetReef(self.drive, self.vision, 8)
        # pc = PlaceCoral(self.periscope, 1)
        # self.scheduler.schedule(tr.andThen(pc))
        # self.scheduler.schedule(pc)

        # def transit():
        #     self.periscope.arm.target = config.transit_setpoint

        # branches = [
        #     (8, 3),
        #     (8, 2),
        #     (8, 1),
        # ]

        # cmds = []
        # for branch in branches:
        #     cmd = (
        #         ApproachHPS(self.drive, self.vision, self.periscope, graph, False)
        #         .andThen(InstantCommand(transit))
        #         .andThen(
        #             ApproachReef(
        #                 self.drive, self.vision, self.periscope.arm, graph, *branch
        #             )
        #         )
        #         .andThen(PlaceCoral(self.periscope))
        #     )
        #     cmds.append(cmd)

        # cmd = InstantCommand(transit).andThen(reduce(Command.andThen, cmds))
        # self.scheduler.schedule(cmd)

        # cmd = (
        #     CollectCoral(self.periscope, False)
        #     .deadlineWith(
        #         ApproachHPS(self.drive, self.vision, self.periscope.arm, graph, False)
        #     )
        #     .andThen(InstantCommand(transit))
        #     .andThen(
        #         ApproachReef(self.drive, self.vision, self.periscope.arm, graph, 11, 2)
        #     )
        #     .andThen(PlaceCoral(self.periscope))
        # )
        # self.scheduler.schedule(cmd)

        g, s = make_auto_methods(self.drive, self.vision, self.periscope, graph)
        # cmds = [
        #     s(11, 3),
        #     g(False),
        #     s(8, 3),
        #     g(False),
        #     s(8, 2),
        # ]
        cmds = [
            s(2, 3),
            g(True),
            s(5, 3),
            g(True),
            s(4, 3),
        ]
        # cmds = [
        #     s(11, 3),
        #     g(False),
        #     s(8, 3),
        #     g(False),
        #     s(11, 2),
        #     g(False),
        #     s(8, 2),
        #     g(False),
        #     s(11, 1),
        #     g(False),
        #     s(8, 1),
        # ]
        self.scheduler.schedule(reduce(Command.andThen, cmds))

    def autonomousPeriodic(self):
        self.autolower()

    def autonomousExit(self):
        pass

    def teleopInit(self):
        self.target_align_cmd = None
        self.right_bumper_toggle = False
        self.left_bumper_toggle = False
        # Remove, because we don't want it retracting after auto
        # in case our arm is hooked with the reef
        # self.periscope.arm.target = config.transit_setpoint
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

    @time_f("teleop periodic")
    def teleopPeriodic(self):
        def deadzone(activation: float) -> float:
            if abs(activation) < 0.01:
                return 0.0
            return activation

        # Driving
        self.field_oriented ^= self.driver_controller.getBackButtonPressed()
        if not (
            self.target_align_cmd is not None and self.target_align_cmd.isScheduled()
        ):
            drive_speed = (
                -config.drive_speed
                if config.is_red() and self.field_oriented
                else config.drive_speed
            )
            drive_input = Transform2d(
                deadzone(-self.driver_controller.getLeftY()) * drive_speed,
                deadzone(-self.driver_controller.getLeftX()) * drive_speed,
                deadzone(-self.driver_controller.getRightX()) * config.turn_speed,
            )
            self.drive.drive(drive_input, self.field_oriented)

        if self.driver_controller.getStartButtonPressed():
            self.drive.odometry.reset(
                Pose2d(Translation2d(), Rotation2d(pi if config.is_red() else 0))
            )

        if self.driver_controller.getPOV() == 180:
            self.drive.chassis.zero_swerves()

        # self.periscope.arm.should_extend = self.driver_controller.getRightBumper()
        if self.driver_controller.getAButtonPressed():
            self.periscope.arm.target = config.transit_setpoint
        elif self.driver_controller.getRightStickButtonPressed():
            self.periscope.arm.target = config.source_setpoint
        elif self.driver_controller.getYButtonPressed():
            self.periscope.arm.target = Transform2d(
                config.ik_neutral_x, config.ik_neutral_y, config.ik_neutral_wrist
            )
        elif self.driver_controller.getRightBumperButtonPressed():
            self.periscope.arm.target = self.manip_controller.arm_setpoint
        elif (
            self.driver_controller.getLeftBumperButtonPressed()
            and self.manip_controller.stalk_selection is not None
        ):
            self.target_align_cmd = TargetReef(
                self.drive, self.vision, self.manip_controller.stalk_selection
            )
            self.scheduler.schedule(self.target_align_cmd)
        elif (
            self.driver_controller.getLeftBumperButtonReleased()
            and self.target_align_cmd is not None
        ):
            self.target_align_cmd.cancel()
            self.target_align_cmd = None

        self.autolower()

        claw_power = (
            self.driver_controller.getLeftTriggerAxis()
            - self.driver_controller.getRightTriggerAxis()
        )
        self.periscope.claw.set(claw_power)

    def teleopExit(self):
        pass

    def testInit(self):
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
        SmartDashboard.putNumber("new IK x", config.ik_neutral_x)
        SmartDashboard.putNumber("new IK y", config.ik_neutral_y)
        units.degreesToRadians(
            SmartDashboard.putNumber(
                "new IK wrist", units.radiansToDegrees(config.ik_neutral_wrist)
            )
        )
        SmartDashboard.putNumber("hff", 0.0065)

    def testPeriodic(self):
        if self.driver_controller.getAButtonPressed():
            self.read_typed_ik_input()
            # entry = list(config.elevator_dynamics_table[5])
            # entry[1] = SmartDashboard.getNumber("hff", 0.0065)
            # config.elevator_dynamics_table[5] = tuple(entry)

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
            "new elevator target", self.periscope.arm.elevator.target_extension
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

    def should_autolower(self):
        return (
            self.near_reef_change == -1
            and not (self.periscope.claw.has_algae() or self.periscope.claw.has_coral())
        ) or (self.near_source_change == -1 and self.periscope.claw.has_coral())

    def autolower(self):
        if self.should_autolower() and self.periscope.arm.target != Transform2d(
            config.ik_neutral_x, config.ik_neutral_y, config.ik_neutral_wrist
        ):
            self.periscope.arm.target = config.transit_setpoint


if __name__ == "__main__":
    wpilib.run(Robot)
