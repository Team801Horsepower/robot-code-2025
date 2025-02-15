#!/usr/bin/env python3

import wpilib
from wpilib.interfaces import GenericHID
from rev import SparkFlex, SparkMax


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.controller = GenericHID(0)

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
        x = [self.controller.getRawButton(i) for i in range(32)]
        print(x)

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
