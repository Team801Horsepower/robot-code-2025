#!/usr/bin/env python3

import wpilib

from wpilib import SmartDashboard

from wpimath.geometry import Transform2d
from commands2 import CommandScheduler

from math import pi

import config
from subsystems import drive, claw, elevator, pivot, wrist

from config import pivot_pid_constraint_constants
from wpimath.trajectory import TrapezoidProfile


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        SmartDashboard.putNumber("elevator_extension", 0)

        SmartDashboard.putNumber("kP", 0)
        SmartDashboard.putNumber("kI", 0)
        SmartDashboard.putNumber("kD", 0)

        SmartDashboard.putNumber("acc_lim", 15)

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
        SmartDashboard.putNumber("elevator extension", self.elevator.get_extension())
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
        target = SmartDashboard.getNumber("elevator_extension", 0)

        kP = SmartDashboard.getNumber("kP", 0)
        kI = SmartDashboard.getNumber("kI", 0)
        kD = SmartDashboard.getNumber("kD", 0)

        acc_lim = SmartDashboard.getNumber("acc_lim", 15)

        SmartDashboard.putNumber(
            "deviation", self.pivot.target_angle - self.pivot.get_angle()
        )

        SmartDashboard.putNumber("pivot target", self.pivot.target_angle)

        self.pivot.target_angle = pi / 2 - (
            self.driver_controller.getLeftTriggerAxis() * 0.7
        )

        self.elevator.target_extension = (
            self.driver_controller.getRightTriggerAxis()
            * (config.extension_range[1] - config.extension_range[0])
            + config.extension_range[0]
        )

        if self.driver_controller.getAButton():
            self.elevator.target_extension = target
            # self.pivot.theta_pid.setP(kP)
            # self.pivot.theta_pid.setI(kI)
            # self.pivot.theta_pid.setD(kD)
            # self.pivot.theta_pid.setConstraints(TrapezoidProfile.Constraints(1000, acc_lim))

    def testExit(self):
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
