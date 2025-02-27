from wpimath.geometry import Transform2d, Translation2d, Rotation2d
from commands2 import Command


from math import tan, pi

from subsystems.drive import Drive
from subsystems.vision import Vision
import config


class StrafeToScore(Command):
    def __init__(self, drive: Drive, vision: Vision):
        self.drive = drive
        self.vision = vision

        self.strafe_pid = config.drive_pid
        self.yaw_pid = config.turn_pid
        self.drive_pid = config.drive_pid

        self.atag_pos = None

        self.should_run = False

        if config.is_red():
            self.side_yaw = 3 * pi / 2
        else:
            self.side_yaw = pi / 2

        self.red_tags = [10, 11, 6, 7, 8, 9]
        self.blue_tags = [21, 20, 19, 18, 17, 22]

    def initialize(self):
        pass

    def execute(self):
        score_pos = 0
        cur_rot = self.drive.odometry.rotation().radians()
        score_atag_cam1 = self.vision.cur_atag(
            1, self.red_tags[int(score_pos / 2)], self.blue_tags[int(score_pos / 2)]
        )
        score_atag_cam2 = self.vision.cur_atag(
            2, self.red_tags[int(score_pos / 2)], self.blue_tags[int(score_pos / 2)]
        )
        target_atag = None
        if score_atag_cam1 is not None and score_atag_cam2 is not None:
            target_atag = (
                (score_atag_cam1[0] + score_atag_cam2[0]) / 2,
                (score_atag_cam1[1] + score_atag_cam2[1]) / 2,
            )

        if target_atag is not None:
            atag_pitch, atag_yaw = target_atag

            robot_dist = (config.score_tag_height - config.front_camera_height) / tan(
                atag_pitch  # + config.camera_angle
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

        if self.atag_pos is not None:
            strafe_power = self.strafe_pid.calculate(
                self.drive.odometry.pose().x, self.atag_pos.x
            )
            drive_power = self.drive_pid.calculate(
                self.drive.odometry.pose().y, self.atag_pos.y
            )

        else:
            strafe_power = 0

        target_yaw = self.side_yaw
        while target_yaw - cur_rot > pi:
            target_yaw -= 2 * pi
        while target_yaw - cur_rot < -pi:
            target_yaw += 2 * pi

        yaw_power = self.yaw_pid.calculate(cur_rot, target_yaw)
        drive_input = Transform2d(drive_power, strafe_power, yaw_power)
        self.drive.drive(drive_input)

    def isFinished(self):
        return False
