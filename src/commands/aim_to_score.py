from wpimath.geometry import Transform2d, Translation2d, Rotation2d
from commands2 import Command
from wpimath.controller import PIDController

from math import tan, pi

from subsystems.drive import Drive
from subsystems.vision import Vision
import config


class StrafeToScore(Command):
    def __init__(self, drive: Drive, vision: Vision, score_pos: int):
        self.drive = drive
        self.vision = vision
        self.score_pos = score_pos

        self.strafe_pid = PIDController(5.0, 0, 0)
        self.yaw_pid = PIDController(5.0, 0, 0)
        self.drive_pid = PIDController(5.0, 0, 0.03)

        self.atag_pos = None

        self.should_run = False

        self.red_tags = [10, 11, 6, 7, 8, 9]
        self.blue_tags = [21, 20, 19, 18, 17, 22]

    def initialize(self):
        pass

    def execute(self):
        cur_rot = self.drive.odometry.rotation().radians()
        score_atag_cam1 = self.vision.cur_atag(
            0,
            self.red_tags[int(self.score_pos / 2)],
            self.blue_tags[int(self.score_pos / 2)],
        )
        score_atag_cam2 = self.vision.cur_atag(
            1,
            self.red_tags[int(self.score_pos / 2)],
            self.blue_tags[int(self.score_pos / 2)],
        )

        side_offset = Translation2d(0, -1)
        side_offset = side_offset.rotateBy(Rotation2d(int(self.score_pos / 2) * pi / 3))
        front_offset = Translation2d(1, 0)
        front_offset = side_offset.rotateBy(
            Rotation2d(int(self.score_pos / 2) * pi / 3)
        )

        if self.score_pos % 2:
            side_offset = side_offset * -0.1524
        else:
            side_offset = side_offset * 0.1524

        target_atag = None
        if score_atag_cam1 is not None and score_atag_cam2 is not None:
            target_atag = (
                (score_atag_cam1[0] + score_atag_cam2[0]) / 2,
                (score_atag_cam1[1] + score_atag_cam2[1]) / 2,
            )

        if target_atag is not None:
            atag_pitch, atag_yaw = target_atag

            robot_dist = (config.score_atag_height - config.front_camera_height) / tan(
                atag_pitch + config.front_camera_pitch
            )
            # robot_dist = cam_dist + config.camera_center_distance

            robot_yaw_diff = -atag_yaw
            # robot_yaw_diff = atan(
            #     # cam_dist / robot_dist *
            #     tan(cam_yaw_diff)
            # )

            self.atag_pos = self.drive.odometry.pose().translation() + Translation2d(
                robot_dist, Rotation2d(cur_rot + robot_yaw_diff)
            )

        if not self.should_run:
            return

        target_x = 0
        target_y = 0

        if self.atag_pos is not None:
            target_x = self.atag_pos.x + side_offset.x + front_offset.x
            target_y = self.atag_pos.y + side_offset.y + front_offset.y
            strafe_power = self.strafe_pid.calculate(
                self.drive.odometry.pose().x, target_x
            )
            drive_power = self.drive_pid.calculate(
                self.drive.odometry.pose().y, target_y
            )

        else:
            strafe_power = 0
            drive_power = 0

        if config.is_red():
            target_yaw = int(self.score_pos / 2) * pi / 3
        else:
            target_yaw = pi + int(self.score_pos / 2) * pi / 3

        while target_yaw - cur_rot > pi:
            target_yaw -= 2 * pi
        while target_yaw - cur_rot < -pi:
            target_yaw += 2 * pi

        if self.atag_pos is not None:
            if (
                abs(target_yaw - cur_rot) < 0.1
                and abs(self.drive.odometry.pose().x - target_x) < 0.1
                and abs(self.drive.odometry.pose().y - target_y) < 0.1
            ):
                self.should_run = False
            yaw_power = self.yaw_pid.calculate(cur_rot, target_yaw)
            drive_input = Transform2d(drive_power, strafe_power, yaw_power)
            self.drive.drive(drive_input)

    def isFinished(self):
        return self.should_run
