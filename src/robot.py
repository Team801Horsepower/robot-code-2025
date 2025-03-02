#!/usr/bin/env python3

import wpilib

from wpilib import SmartDashboard

from wpimath.geometry import Transform2d
from commands2 import CommandScheduler
from wpimath import units

import config
from subsystems import drive, periscope


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        # SmartDashboard.putNumber("elevator_extension", 0)

        # SmartDashboard.putNumber("kP", 0)
        # SmartDashboard.putNumber("kI", 0)
        # SmartDashboard.putNumber("kD", 0)

        # SmartDashboard.putNumber("acc_lim", 15)

        self.scheduler = CommandScheduler()

        # Subsystem initialization
        self.scheduler.unregisterAllSubsystems()

        self.drive = drive.Drive(self.scheduler)
        # self.scheduler.registerSubsystem(self.drive)

        self.periscope = periscope.Periscope(self.scheduler, self.drive.odometry.ahrs)
        # self.arm = arm.Arm(self.scheduler)
        # self.wrist = wrist.Wrist(self.scheduler)
        # self.scheduler.registerSubsystem(self.claw)
        # self.climber = climber.Climber(self.scheduler)
        # self.elevator = elevator.Elevator(self.scheduler)
        # self.scheduler.registerSubsystem(self.elevator)
        # self.pivot = pivot.Pivot(
        #     self.scheduler, self.elevator, self.drive.odometry.ahrs
        # )
        # self.scheduler.registerSubsystem(self.pivot)
        # self.turn_signals = turn_signals.TurnSignals(self.scheduler)

        self.driver_controller = wpilib.XboxController(0)
        self.manip_controller = wpilib.XboxController(1)

        self.drive.chassis.set_swerves()

        self.field_oriented = True

        for encoder in self.periscope.arm.elevator.extension_motor_encoders:
            encoder.setPosition(0)

    def robotPeriodic(self):
        # This line must always be present in robotPeriodic, or else
        # commands and subsystem periodic methods will not run
        self.scheduler.run()

        SmartDashboard.putNumber("pivot angle", self.periscope.arm.pivot.get_angle())
        SmartDashboard.putNumber(
            "elevator extension", self.periscope.arm.elevator.get_extension()
        )
        SmartDashboard.putNumber(
            "wrist pos",
            units.radiansToDegrees(self.periscope.arm.wrist.angle()),
        )

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
        self.periscope.arm.target = Transform2d(
            config.ik_neutral_x, config.ik_neutral_y, config.ik_neutral_wrist
        )

        SmartDashboard.putNumber("new IK x", config.ik_neutral_x)
        SmartDashboard.putNumber("new IK y", config.ik_neutral_y)
        SmartDashboard.putNumber(
            "new IK wrist", units.radiansToDegrees(config.ik_neutral_wrist)
        )

        SmartDashboard.putNumber(
            "new pivot target",
            units.radiansToDegrees(self.periscope.arm.pivot.target_angle),
        )
        SmartDashboard.putNumber(
            "new elevator target", self.periscope.arm.elevator.target_extension
        )
        SmartDashboard.putNumber(
            "new wrist target",
            units.radiansToDegrees(self.periscope.arm.wrist.target_angle),
        )

    def teleopPeriodic(self):
        def deadzone(activation: float) -> float:
            if abs(activation) < 0.01:
                return 0.0
            return activation

        self.field_oriented ^= self.driver_controller.getBButtonPressed()
        drive_input = Transform2d(
            deadzone(-self.driver_controller.getLeftY()) * config.drive_speed,
            deadzone(-self.driver_controller.getLeftX()) * config.drive_speed,
            deadzone(-self.driver_controller.getRightX()) * config.turn_speed,
        )
        self.drive.drive(drive_input, self.field_oriented)

        if self.driver_controller.getBackButtonPressed():
            self.drive.chassis.zero_swerves()

        if self.driver_controller.getAButtonPressed():
            self.read_typed_ik_input()

        self.periscope.arm.should_extend ^= self.driver_controller.getXButtonPressed()

        claw_power = (
            self.driver_controller.getLeftTriggerAxis()
            - self.driver_controller.getRightTriggerAxis()
        )
        self.periscope.claw.set(claw_power)

    def teleopExit(self):
        pass

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def testExit(self):
        pass

    def read_typed_ik_input(self):
        new_ik_x = SmartDashboard.getNumber("new IK x", config.ik_neutral_x)
        new_ik_y = SmartDashboard.getNumber("new IK y", config.ik_neutral_y)
        new_ik_wrist = units.degreesToRadians(
            SmartDashboard.getNumber(
                "new IK wrist", units.radiansToDegrees(config.ik_neutral_wrist)
            )
        )
        self.periscope.arm.target = Transform2d(new_ik_x, new_ik_y, new_ik_wrist)

    def read_typed_arm_input(self):
        new_pivot_target = units.degreesToRadians(
            SmartDashboard.getNumber(
                "new pivot target",
                units.radiansToDegrees(self.periscope.arm.pivot.target_angle),
            )
        )
        new_extension_target = SmartDashboard.getNumber(
            "new elevator target", self.periscope.arm.pivot.target_angle
        )
        new_wrist_target = units.degreesToRadians(
            SmartDashboard.getNumber(
                "new wrist target",
                units.radiansToDegrees(self.periscope.arm.wrist.target_angle),
            )
        )
        self.periscope.arm.target = (
            new_pivot_target,
            new_extension_target,
            new_wrist_target,
        )


if __name__ == "__main__":
    wpilib.run(Robot)
