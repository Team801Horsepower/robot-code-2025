#!/usr/bin/env python3

import wpilib
from rev import CANSparkFlex, CANSparkMax

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.drive_motor = CANSparkFlex(5, CANSparkFlex.MotorType.kBrushless)
        self.turn_motor = CANSparkMax(3, CANSparkFlex.MotorType.kBrushless)

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
