#!/usr/bin/env python3

import wpilib
from wpilib import SmartDashboard
from wpimath.geometry import Transform2d
from commands2 import CommandScheduler

import config
from subsystems import drive
from subsystems import vision


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.driver_controller = wpilib.XboxController(0)
        self.manip_controller = wpilib.XboxController(1)

        self.scheduler = CommandScheduler()

        self.drive = drive.Drive(self.scheduler)
        self.drive.chassis.set_swerves()

        self.vision = vision.Vision(self.scheduler)

        self.field_oriented = True

    def robotPeriodic(self):
        SmartDashboard.putNumber("heading", self.drive.odometry.rotation().degrees())

        estimates = self.vision.estimate_multitag_pose(
            self.drive.odometry.rotation().radians()
        )
        for i, (pose, conf) in enumerate(estimates):
            SmartDashboard.putNumber(f"x {i}", pose.x)
            SmartDashboard.putNumber(f"y {i}", pose.y)
            SmartDashboard.putNumber(f"confidence {i}", conf)

        self.vision.test()

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass

    def disabledExit(self):
        pass

    def autonomousInit(self):
        pass

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
            self.drive.odometry.reset()

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
