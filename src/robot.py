#!/usr/bin/env python3

import wpilib

from wpilib import SmartDashboard

from wpimath.geometry import Transform2d
from commands2 import CommandScheduler

from math import pi

import config
from subsystems import drive, claw, elevator, pivot, wrist


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        SmartDashboard.putNumber("pvt_theta", 1.57)
        self.scheduler = CommandScheduler()

        # Subsystem initialization
        self.scheduler.unregisterAllSubsystems()

        self.claw = claw.Claw(self.scheduler)

        self.drive = drive.Drive(self.scheduler)
        # self.scheduler.registerSubsystem(self.drive)

        # self.periscope = periscope.Periscope(self.scheduler, self.drive.odometry.ahrs)
        # self.arm = arm.Arm(self.scheduler)
        self.wrist = wrist.Wrist(self.scheduler)
        # self.scheduler.registerSubsystem(self.claw)
        # self.climber = climber.Climber(self.scheduler)
        self.elevator = elevator.Elevator(self.scheduler)
        # self.scheduler.registerSubsystem(self.elevator)
        self.pivot = pivot.Pivot(
            self.scheduler, self.elevator, self.drive.odometry.ahrs
        )
        # self.scheduler.registerSubsystem(self.pivot)
        # self.turn_signals = turn_signals.TurnSignals(self.scheduler)

        self.driver_controller = wpilib.XboxController(0)
        self.manip_controller = wpilib.XboxController(1)

        self.drive.chassis.set_swerves()

        for encoder in self.elevator.extension_motor_encoders:
            encoder.setPosition(0)

    def robotPeriodic(self):
        self.scheduler.run()
        SmartDashboard.putNumber("Pivot Angle", self.pivot.get_angle())
        SmartDashboard.putNumber(
            "elevator extension",
            self.elevator.extension_motor_encoders[0].getPosition(),
        )
        SmartDashboard.putNumber("wrist pos", self.wrist.wrist_encoder.getPosition())

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
        target = SmartDashboard.getNumber("pvt_theta", pi / 2)

        self.pivot.target_angle = 1.57 - (self.driver_controller.getLeftTriggerAxis() * 0.7)


    def testExit(self):
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
