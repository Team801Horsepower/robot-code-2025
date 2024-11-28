#!/usr/bin/env python3

import wpilib
from rev import CANSparkFlex, CANSparkMax

from phoenix6.hardware import cancoder


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.ccs = [cancoder.CANcoder(n) for n in (12, 22, 32, 42)]

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
        for cc in self.ccs:
            print(cc.get_position(), end="\t")
            print()

    def autonomousExit(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        self.drive_motor.set(0.05)
        self.turn_motor.set(0.05)

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
