#!/usr/bin/env python3

import wpilib
from rev import SparkFlex, SparkMax


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.flexes = [SparkFlex(n, SparkFlex.MotorType.kBrushless) for n in (10, 20, 30, 40)]
        self.maxes = [SparkMax(n, SparkMax.MotorType.kBrushless) for n in (11, 21, 31, 41)]

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
        # for motor in self.flexes:
        #     motor.set(0.5)
        # for motor in self.maxes:
        #     motor.set(0.5)

        self.maxes[0].set(0.5)

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
