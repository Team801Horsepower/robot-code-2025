#!/usr/bin/env python3

import wpilib
from wpimath.geometry import Transform2d
from commands2 import CommandScheduler

import config
from subsystems import drive


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.driver_controller = wpilib.XboxController(0)
        self.manip_controller = wpilib.XboxController(1)

        self.scheduler = CommandScheduler()

        self.drive = drive.Drive(self.scheduler)

    def robotPeriodic(self):
        pass

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
        self.drive.drive(drive_input)

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
