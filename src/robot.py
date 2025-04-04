#!/usr/bin/env python3

import wpilib
from wpilib import SmartDashboard, DataLogManager
from wpilib.interfaces import GenericHID
from wpimath.geometry import Transform2d, Pose2d, Rotation2d, Translation2d
from commands2 import CommandScheduler, Command, InstantCommand, WaitCommand
from wpimath import units
from functools import reduce
from math import sin, pi
import time

import config
from subsystems import drive, periscope, vision, manipulator_controller, turn_signals

from commands.approach_reef import ApproachReef
from commands.approach_hps import ApproachHPS
from commands.target_reef import TargetReef
from commands.continuous import Continuous

from utils.graph import Graph
from utils import time_f, letter_to_morse

from auto_actions import make_auto_methods


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        DataLogManager.start()
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

        self.turn_signals = turn_signals.TurnSignals(
            self.scheduler, self.manip_controller, self.periscope.claw
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

        # SmartDashboard.putNumber("auto drive speed", config.auto_drive_speed)
        # SmartDashboard.putNumber("auto turn speed", config.auto_turn_speed)
        self.morse_cmd = None

        graph_path = config.code_path + "graph.json"
        self.graph = Graph(graph_path)

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

        SmartDashboard.putNumber("reef approach P", 8.0)
        SmartDashboard.putNumber("reef strafe P", 4.5)
        SmartDashboard.putNumber("reef approach I", 0.0)
        SmartDashboard.putNumber("reef strafe I", 0.0)
        SmartDashboard.putNumber("reef approach D", 0.0)
        SmartDashboard.putNumber("reef strafe D", 0.0)
        SmartDashboard.putNumber("reef align speed", 4.0)
        SmartDashboard.putNumber("align threshold", 1.2)

        SmartDashboard.putBoolean("start auto on left", False)

        SmartDashboard.putString("morse message", "801!")

    @time_f("periodic robot")
    def robotPeriodic(self):
        # This line must always be present in robotPeriodic, or else
        # commands and subsystem periodic methods will not run
        self.scheduler.run()

        SmartDashboard.putNumberArray(
            "drive encoder pos",
            [
                swerve.drive_encoder.getPosition()
                for swerve in self.drive.chassis.swerves
            ],
        )
        SmartDashboard.putNumberArray(
            "drive encoder vel",
            [
                swerve.drive_encoder.getVelocity()
                for swerve in self.drive.chassis.swerves
            ],
        )
        SmartDashboard.putNumberArray(
            "drive motor current",
            [
                swerve.drive_motor.getOutputCurrent()
                for swerve in self.drive.chassis.swerves
            ],
        )

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

        ests_th = 4
        conf_th = 0.4
        dev_th = 0.05

        n_ests, report = self.vision.pos_report(heading, ests_th, conf_th)
        SmartDashboard.putNumber("pos estimates", n_ests)
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
            if (
                n_ests >= ests_th
                and conf >= conf_th
                and dev < dev_th
                # TODO: should we have this?
                and not self.isAutonomousEnabled()
            ):
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

        # tag8_pose = self.vision.layout.getTagPose(8)
        # if tag8_pose is not None:
        #     tag8_pos = tag8_pose.toPose2d().translation()
        #     fr_swerve_pos = robot_pos + Translation2d(
        #         config.robot_dimensions.x, -config.robot_dimensions.y
        #     ).rotateBy(robot_pose.rotation())
        #     dist = (fr_swerve_pos - tag8_pos).norm()
        #     SmartDashboard.putNumber("tag8-FR swerve (in)", units.metersToInches(dist))
        if (
            self.morse_cmd is not None and not self.morse_cmd.isScheduled()
        ) or self.morse_cmd is None:
            self.turn_signals.should_turn_signal = True

        now = time.time()
        took = now - self.last_loop
        self.last_loop = now
        SmartDashboard.putNumber("loop time", took)

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass

    def disabledExit(self):
        pass

    def autonomousInit(self):
        self.auto_start_time = time.time()

        self.periscope.arm.pivot.has_flipped_middle_finger = False
        self.periscope.arm.set_target(config.transit_setpoint)

        left_start = SmartDashboard.getBoolean("start auto on left", False)

        start_y = 0.5
        if left_start:
            start_y = config.field_width - start_y
        self.drive.odometry.reset(
            Pose2d(
                config.flip_red(Translation2d(7.17, start_y)),
                Rotation2d(pi if config.is_red() else 0),
            )
        )

        g, s = make_auto_methods(self.drive, self.vision, self.periscope, self.graph)

        if left_start:
            cmds = [
                s(3, 3),
                g(True),
                s(4, 3),
                g(True),
                s(5, 3),
            ]
        else:
            cmds = [
                s(10, 3),
                g(False),
                s(9, 3),
                g(False),
                s(8, 3),
            ]

        # cmds = [
        #     # s(11, 3),
        #     s(11, 2),
        #     g(False),
        #     # s(8, 3),
        #     s(9, 2),
        #     g(False),
        #     s(8, 2),
        # ]

        # cmds = [
        #     s(2, 3),
        #     g(True),
        #     s(5, 3),
        #     g(True),
        #     s(4, 3),
        # ]

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
        def log_time():
            auto_took = time.time() - self.auto_start_time
            SmartDashboard.putNumber("auto took", auto_took)

        self.scheduler.schedule(
            reduce(Command.andThen, cmds).andThen(InstantCommand(log_time))
        )

    def autonomousPeriodic(self):
        self.autolower()

    def autonomousExit(self):
        pass

    def teleopInit(self):
        self.target_align_cmd = None
        self.right_bumper_toggle = False
        self.left_bumper_toggle = False

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

        # FIXME: Takes several hundred milliseconds for some reason
        # self.morse_cmd = self.blink_morse("Horsepower")

    @time_f("periodic teleop")
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
            self.periscope.arm.set_target(config.transit_setpoint)
        elif self.driver_controller.getRightStickButtonPressed():
            self.periscope.arm.set_target(config.source_setpoint)
        elif self.driver_controller.getYButtonPressed():
            self.periscope.arm.set_target(
                Transform2d(
                    config.ik_neutral_x, config.ik_neutral_y, config.ik_neutral_wrist
                )
            )
        elif self.driver_controller.getRightBumperButtonPressed():
            self.periscope.arm.set_target(self.manip_controller.arm_setpoint)
        elif (
            self.driver_controller.getLeftBumperButtonPressed()
            and self.manip_controller.stalk_selection is not None
        ):
            if self.target_align_cmd is not None:
                self.target_align_cmd.cancel()

            if self.manip_controller.disable_pathfinding:
                self.target_align_cmd = Continuous(
                    TargetReef(
                        self.drive,
                        self.vision,
                        self.manip_controller.stalk_selection,
                        self.manip_controller.reef_algae_selected,
                    )
                )
            else:
                self.target_align_cmd = Continuous(
                    ApproachReef(
                        self.drive,
                        self.vision,
                        self.periscope.arm,
                        self.graph,
                        self.manip_controller.stalk_selection,
                        self.manip_controller.target_level or 0,
                        self.manip_controller.reef_algae_selected,
                    )
                )

            self.scheduler.schedule(self.target_align_cmd)
        elif not self.manip_controller.climb_mode and (
            (left_hps := self.driver_controller.getXButtonPressed())
            or self.driver_controller.getBButtonPressed()
        ):
            if self.target_align_cmd is not None:
                self.target_align_cmd.cancel()
            self.target_align_cmd = Continuous(
                ApproachHPS(
                    self.drive, self.vision, self.periscope, self.graph, left_hps
                )
            )
            self.scheduler.schedule(self.target_align_cmd)
        elif self.manip_controller.climb_mode and (
            (climb_down := self.driver_controller.getXButtonPressed())
            or self.driver_controller.getBButtonPressed()
        ):
            self.periscope.arm.set_target(
                config.climb_lowered_setpoint
                if climb_down
                else config.climb_raised_setpoint
            )
        elif (
            self.driver_controller.getLeftBumperButtonReleased()
            or self.driver_controller.getXButtonReleased()
            or self.driver_controller.getBButtonReleased()
        ) and self.target_align_cmd is not None:
            self.target_align_cmd.cancel()
            self.target_align_cmd = None

        self.autolower()

        if self.manip_controller.climb_mode:
            if self.target_align_cmd is not None:
                self.target_align_cmd.cancel()
                self.target_align_cmd = None
            self.periscope.claw.set(0)

            self.periscope.climber.climb(
                max(
                    self.driver_controller.getLeftTriggerAxis(),
                    self.driver_controller.getRightTriggerAxis(),
                )
            )
        else:
            self.periscope.climber.climb(0)

            if not (
                self.target_align_cmd is not None
                and isinstance(self.target_align_cmd.inner, ApproachHPS)
                and self.target_align_cmd.isScheduled()
                # self.target_align_cmd is not None
                # and self.target_align_cmd.isScheduled()
            ):
                claw_power = (
                    self.driver_controller.getLeftTriggerAxis()
                    - self.driver_controller.getRightTriggerAxis()
                )
                self.periscope.claw.set(claw_power)

        should_rumble = (
            (
                self.driver_controller.getRightTriggerAxis() > 0.1
                or self.driver_controller.getXButton()
                or self.driver_controller.getBButton()
            )
            and self.periscope.claw.has_coral()
        ) or (
            self.driver_controller.getLeftBumperButton()
            and self.target_align_cmd is not None
            and self.target_align_cmd.inner_finished()
        )
        self.driver_controller.setRumble(
            GenericHID.RumbleType.kBothRumble, 1 if should_rumble else 0
        )

    def teleopExit(self):
        self.driver_controller.setRumble(GenericHID.RumbleType.kBothRumble, 0)

    def testInit(self):
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
        self.periscope.arm.set_target(Transform2d(new_ik_x, new_ik_y, new_ik_wrist))

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
        self.periscope.arm.set_target(
            (
                new_pivot_target,
                new_extension_target,
                new_wrist_target,
            )
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
            self.periscope.arm.set_target(config.transit_setpoint)

    @time_f("blink_morse")
    def blink_morse(self, message):
        def flip_turn_signals(value):
            self.turn_signals.signal(1, value)

        blink_string = message
        led_cmds = []
        for letter in blink_string:
            for blink_time in letter_to_morse(letter):
                led_cmds.append(
                    InstantCommand(lambda: flip_turn_signals(True)).andThen(
                        WaitCommand(blink_time * 0.3)
                    )
                )
                led_cmds.append(
                    InstantCommand(lambda: flip_turn_signals(False)).andThen(
                        WaitCommand(0.3)
                    )
                )
        self.scheduler.schedule(cmd := reduce(Command.andThen, led_cmds))
        self.turn_signals.should_turn_signal = False
        return cmd


if __name__ == "__main__":
    wpilib.run(Robot)
