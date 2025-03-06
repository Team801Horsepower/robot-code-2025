from photonlibpy.photonCamera import PhotonCamera

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.geometry import (
    Transform3d,
    Rotation2d,
    Rotation3d,
    Translation2d,
    Translation3d,
)
from wpimath import units
from wpilib import SmartDashboard
from commands2 import Subsystem, CommandScheduler
from typing import Tuple, Optional, List
import numpy as np

from math import sin, cos, pi, atan, isfinite

import config


class Vision(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        self.cameras: List[Tuple[PhotonCamera, Transform3d]] = [
            (
                PhotonCamera("FrontLeft"),
                Transform3d(
                    Translation3d(-0.11146, 0.2346, 0.27491),
                    Rotation3d(
                        0, units.degreesToRadians(20), units.degreesToRadians(340)
                    ),
                ),
            ),
            (
                PhotonCamera("BackLeft"),
                Transform3d(
                    Translation3d(-0.16406, 0.25998, 0.277),
                    Rotation3d(
                        0, units.degreesToRadians(25), units.degreesToRadians(150)
                    ),
                ),
            ),
            (
                PhotonCamera("FrontRight"),
                Transform3d(
                    Translation3d(-0.11146, -0.2346, 0.27491),
                    Rotation3d(
                        0, units.degreesToRadians(20), units.degreesToRadians(20)
                    ),
                ),
            ),
            (
                PhotonCamera("BackRight"),
                Transform3d(
                    Translation3d(-0.16406, -0.25998, 0.277),
                    Rotation3d(
                        0, units.degreesToRadians(25), units.degreesToRadians(210)
                    ),
                ),
            ),
        ]

        # layout_path = config.code_path + "firehouse-apriltags.json"
        # layout_path = config.code_path + "reefscape-apriltags.json"

        # self.layout = AprilTagFieldLayout(layout_path)
        self.layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)

        scheduler.registerSubsystem(self)

    def periodic(self):
        pass

    # (tag pitch, tag yaw)
    def cur_atag(
        self, cam_i: int, red_id: int, blue_id: int
    ) -> Optional[Tuple[float, float]]:
        atag_id = red_id if config.is_red() else blue_id
        result = self.cameras[cam_i][0].getLatestResult()
        for target in result.getTargets():
            if target.fiducialId == atag_id:
                return (
                    units.degreesToRadians(target.getPitch()),
                    units.degreesToRadians(target.getYaw()),
                )
        return None

    def test(self):
        # atag0 = self.cur_atag(0, 9, 9)
        # if atag0 is not None:
        #     SmartDashboard.putNumber(
        #         "front cam tag 9 yaw", units.radiansToDegrees(atag0[1])
        #     )
        # atag1 = self.cur_atag(1, 9, 9)
        # if atag1 is not None:
        #     SmartDashboard.putNumber(
        #         "rear cam tag 9 yaw", units.radiansToDegrees(atag1[1])
        #     )
        pass

    def estimate_multitag_pos(
        self, robot_angle: float
    ) -> Tuple[int, List[Tuple[Tuple[int, int], Tuple[Translation2d, float]]]]:
        tag_info = []
        for cam, transform in self.cameras:
            res = cam.getLatestResult()
            # print(res)
            tags = res.getTargets()
            for tag in tags:
                id = tag.getFiducialId()
                if self.layout.getTagPose(id) is not None:
                    tag_info.append(
                        (
                            id,
                            units.degreesToRadians(-tag.getYaw())
                            + transform.rotation().toRotation2d().radians(),
                            transform.translation()
                            .toTranslation2d()
                            .rotateBy(Rotation2d(robot_angle)),
                        )
                    )
        positions = []
        for i in range(len(tag_info)):
            for j in range(i + 1, len(tag_info)):
                positions.append(
                    (
                        (tag_info[i][0], tag_info[j][0]),
                        self.estimate_2tag_pos(robot_angle, tag_info[i], tag_info[j]),
                    )
                )
        return (len(tag_info), positions)

    def estimate_2tag_pos(
        self,
        robot_angle: float,
        # ID, yaw angle, camera offset from robot origin
        tag1: Tuple[int, float, Translation2d],
        tag2: Tuple[int, float, Translation2d],
    ) -> Tuple[Translation2d, float]:
        p1 = self.layout.getTagPose(tag1[0]).toPose2d().translation()
        p2 = self.layout.getTagPose(tag2[0]).toPose2d().translation()

        th1 = tag1[1] + robot_angle
        th2 = tag2[1] + robot_angle
        confidence = sin(th1 - th2) ** 2

        a1, b1 = -sin(th1), cos(th1)
        a2, b2 = -sin(th2), cos(th2)

        camera_delta = tag2[2] - tag1[2]

        c1 = a1 * p1.x + b1 * p1.y
        c2 = a2 * p2.x + b2 * p2.y - a2 * camera_delta.x - b2 * camera_delta.y

        x = (b2 * c1 - b1 * c2) / (a1 * b2 - a2 * b1)
        y = (c2 * a1 - c1 * a2) / (a1 * b2 - a2 * b1)

        return (
            Translation2d(x, y) - tag1[2],
            confidence,
        )

    def pos_report(
        self, robot_angle: float
    ) -> Tuple[int, Optional[Tuple[Translation2d, float, float]]]:
        ntags, estimates = self.estimate_multitag_pos(robot_angle)
        confs = [tup[1][1] for tup in estimates]
        total_conf = sum(confs)
        weights = [conf / total_conf for conf in confs]
        positions = [tup[1][0] for tup in estimates]

        try:
            avg_pos = Translation2d(np.dot(weights, positions))
            sq_diffs = [(pos - avg_pos).norm() ** 2 for pos in positions]
            # deviation_conf = 1 / (1 + float(np.sqrt(np.dot(weights, sq_diffs))))
            deviation = float(np.sqrt(np.dot(weights, sq_diffs)))
            composite_conf = float(np.dot(weights, confs))
            return (ntags, (avg_pos, composite_conf, deviation))
        except TypeError:
            # avg_pos = None
            # deviation_conf = 0
            # composite_conf = 0
            return (ntags, None)

    def heading_correction(self, robot_angle: float) -> Optional[float]:
        x1 = robot_angle - pi / 4
        x2 = robot_angle + pi / 4
        try:
            if self.pos_report(robot_angle)[1][2] == 0:
                return None
            y1 = self.pos_report(x1)[1][2]
            y2 = self.pos_report(x2)[1][2]
        except TypeError:
            return None

        mat = np.array([[cos(x1), sin(x1)], [cos(x2), sin(x2)]])
        matinv = np.linalg.inv(mat)

        out1 = np.array([[y1], [y2]])
        out2 = np.array([[y1], [-y2]])

        coefs1 = (matinv @ out1).flatten()
        coefs2 = (matinv @ out2).flatten()

        h1 = round(robot_angle / pi) * pi - atan(coefs1[0] / coefs1[1])
        h2 = round(robot_angle / pi) * pi - atan(coefs2[0] / coefs2[1])

        new_angle = min(h1, h2, key=lambda h: abs(h - robot_angle))
        if isfinite(new_angle):
            return new_angle
        else:
            return None
