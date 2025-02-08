#!/usr/bin/env python3

import wpilib

from wpimath.controller import ProfiledPIDController
from math import pi, sin
from wpimath.trajectory import TrapezoidProfile
from rev import SparkFlex, SparkBaseConfig
from wpimath.geometry import Transform2d
from commands2 import CommandScheduler

import config
from subsystems import drive


class Robot(wpilib.TimedRobot):
    def robotInit(self):

        self.motor = SparkFlex(52, SparkFlex.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()
        self.zero_point = 0

        self.pid = ProfiledPIDController(2, 0.5, 0.025, TrapezoidProfile.Constraints(1, 1))
        self.pid_setpoint = 0.0

        self.driver_controller = wpilib.XboxController(0)
        self.manip_controller = wpilib.XboxController(1)

        self.scheduler = CommandScheduler()

        # self.elevator = elevator.Elevator(self.scheduler)

        self.drive = drive.Drive(self.scheduler)
        self.drive.chassis.set_swerves()

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
        self.zero_point = self.encoder.getPosition() * (2 * pi / 16)

    def testPeriodic(self):
        theta = ((self.encoder.getPosition() * (2 * pi / 16)) - self.zero_point)
        Cm = 9.81 * 1.5
        Ct = -0.00225
        Tg = Cm * sin(theta) * Ct


        self.pid_setpoint = self.driver_controller.getLeftY()

        pid_out = self.pid.calculate(theta, self.pid_setpoint)

        print(theta, '\t', Tg, pid_out)
        self.motor.set(Tg + pid_out)

    def testExit(self):
        self.motor.set(0)


if __name__ == "__main__":
    wpilib.run(Robot)
