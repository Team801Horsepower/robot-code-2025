#!/usr/bin/env python3

import wpilib
from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Transform2d, Translation2d, Pose2d, Rotation2d
from wpimath import units
from commands2 import CommandScheduler
from math import pi, sqrt
import time
from copy import deepcopy

import config
from subsystems import drive
from subsystems import vision

from commands.drive_to_pose import DriveToPose


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.driver_controller = wpilib.XboxController(0)
        self.manip_controller = wpilib.XboxController(1)

        self.scheduler = CommandScheduler()

        self.drive = drive.Drive(self.scheduler)
        self.drive.chassis.set_swerves()

        self.vision = vision.Vision(self.scheduler)

        self.field_oriented = True

        self.log = []

    def robotPeriodic(self):
        # This must be called every cycle, or subsystem periodics and scheduled commands will not run
        self.scheduler.run()

        SmartDashboard.putNumber("heading", self.drive.odometry.rotation().degrees())

        heading = self.drive.odometry.rotation().radians()
        ntags, estimates = self.vision.estimate_multitag_pos(heading)
        for i, ((id1, id2), (pos, conf)) in enumerate(estimates):
            SmartDashboard.putNumber(f"x {i}", units.metersToInches(pos.x))
            SmartDashboard.putNumber(f"y {i}", units.metersToInches(pos.y))
            SmartDashboard.putNumber(f"confidence {i}", conf)
            SmartDashboard.putString(f"pair {i}", f"{id1}–{id2}")

        # confs = [tup[1][1] for tup in estimates]
        # total_conf = sum(confs)
        # weights = [conf / total_conf for conf in confs]
        # positions = [tup[1][0] for tup in estimates]

        # try:
        #     avg_pos = Translation2d(np.dot(weights, positions))
        #     sq_diffs = [(pos - avg_pos).norm() ** 2 for pos in positions]
        #     deviation_conf = 1 / (1 + float(np.sqrt(np.dot(weights, sq_diffs))))
        #     composite_conf = float(np.dot(weights, confs))
        # except TypeError:
        #     avg_pos = None
        #     deviation_conf = 0
        #     composite_conf = 0
        # if avg_pos is not None:
        #     SmartDashboard.putNumber("avg x", units.metersToInches(avg_pos.x))
        #     SmartDashboard.putNumber("avg y", units.metersToInches(avg_pos.y))
        # SmartDashboard.putNumber("deviation conf", deviation_conf)
        # SmartDashboard.putNumber("composite conf", composite_conf)
        # SmartDashboard.putNumber("product conf", deviation_conf * composite_conf)

        ntags = pos = conf = dev = heading_correction = None

        ntags, report = self.vision.pos_report(heading)
        if report is not None:
            pos, conf, dev = report
            SmartDashboard.putNumber("avg x", units.metersToInches(pos.x))
            SmartDashboard.putNumber("avg y", units.metersToInches(pos.y))
            SmartDashboard.putNumber("composite conf", conf)
            SmartDashboard.putNumber("deviation", dev)

            heading_correction = self.vision.heading_correction(heading)
            if heading_correction is not None:
                SmartDashboard.putNumber(
                    "corrected heading", units.radiansToDegrees(heading_correction)
                )
            speeds = self.drive.chassis.chassis_speeds()
            speed = sqrt(speeds.vx**2 + speeds.vy**2)
            if conf >= 0.25 and speed < 0.15:
                # if conf >= 0.2:
                if heading_correction is not None and abs(
                    heading_correction - heading
                ) < units.degreesToRadians(5):
                    # self.drive.odometry.reset_heading(Rotation2d(heading_correction))
                    pass
                self.drive.odometry.reset_translation(pos)
                pass

        robot_pos = self.drive.odometry.pose().translation()
        SmartDashboard.putNumber("robot x", robot_pos.x)
        SmartDashboard.putNumber("robot y", robot_pos.y)

        if pos is None:
            vis_x = vis_y = None
        else:
            vis_x = pos.x
            vis_y = pos.y

        if DriverStation.isEnabled():
            entry = (
                time.time(),
                ntags,
                robot_pos.x,
                robot_pos.y,
                heading,
                conf,
                dev,
                vis_x,
                vis_y,
                heading_correction,
            )
            self.log.append(entry)

        self.vision.test()

    def disabledInit(self):
        log_s = "time,ntags,robot_x,robot_y,heading,confidence,deviation,vision_x,vision_y,heading_correction\n"
        for entry in self.log:
            log_s += ",".join(str(v) for v in entry)
            log_s += "\n"

        now = round(time.time())
        # with open(f"/home/lvuser/logs/log_{now}.csv", "a") as f:
        #     f.write(log_s)

    def disabledPeriodic(self):
        pass

    def disabledExit(self):
        pass

    def autonomousInit(self):
        origin_pose = Pose2d(
            units.inchesToMeters(16 + 12.125),
            units.inchesToMeters(-74.75 + 12.125),
            0,
        )

        other_pose = Pose2d(-0.844, -1.31, -2.31)

        # auto_cmd = DriveToPose(
        #     Pose2d(
        #         units.inchesToMeters(16 + 12.125),
        #         units.inchesToMeters(-74.75 + 12.125),
        #         0,
        #     ),
        #     self.drive,
        # )
        # origin_dtp = DriveToPose(origin_pose, self.drive)
        def f():
            return DriveToPose(origin_pose, self.drive)

        # other_dtp = DriveToPose(other_pose, self.drive)
        def h():
            return DriveToPose(other_pose, self.drive)

        # auto_cmd = origin_dtp.andThen(other_dtp).repeatedly()
        auto_cmd = f().andThen(h()).andThen(f()).andThen(h()).andThen(f()).andThen(h())
        self.scheduler.schedule(auto_cmd)

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        def deadzone(activation: float) -> float:
            if abs(activation) < 0.01:
                return 0.0
            return activation

        drive_input = Transform2d(
            deadzone(-self.driver_controller.getLeftY()) * config.drive_speed,
            deadzone(-self.driver_controller.getLeftX()) * config.drive_speed,
            deadzone(-self.driver_controller.getRightX()) * config.turn_speed,
        )
        self.drive.drive(drive_input, self.field_oriented)

        self.field_oriented ^= self.driver_controller.getBButtonPressed()

        if self.driver_controller.getStartButtonPressed():
            self.drive.odometry.reset(Pose2d(0, 0, pi))

        if self.driver_controller.getBackButtonPressed():
            self.drive.odometry.reset(
                Pose2d(
                    self.drive.odometry.pose().translation(),
                    self.drive.odometry.pose().rotation() + Rotation2d.fromDegrees(1),
                )
            )

    def teleopExit(self):
        pass

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def testExit(self):
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
