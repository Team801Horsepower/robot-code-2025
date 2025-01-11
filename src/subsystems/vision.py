from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from photonlibpy.photonPoseEstimator import (
    PhotonPoseEstimator,
    AprilTagFieldLayout,
    PoseStrategy,
)
from wpimath.geometry import (
    Transform2d,
    Transform3d,
    Rotation2d,
    Rotation3d,
    Pose2d,
    Translation2d,
    Translation3d,
)
from wpimath import units
from commands2 import Subsystem, CommandScheduler
from typing import Tuple, Optional, List

from math import tan, sin, cos

import config


class Vision(Subsystem):
    def __init__(self, scheduler: CommandScheduler, camera_name="Camera_Module_v1"):
        # TODO: remove this and the function parameter
        self.camera = PhotonCamera(camera_name)

        self.cameras: List[Tuple[PhotonCamera, Transform3d]] = [
            (
                PhotonCamera("Front Camera"),
                Transform3d(
                    Translation3d(0.60880625, 0.29845, 0.428625),
                    Rotation3d(0, units.degreesToRadians(-5), 0),
                ),
            ),
            (
                PhotonCamera("Rear Camera"),
                Transform3d(
                    Translation3d(0.52466875, 0.29845, 0.428625),
                    Rotation3d(0, units.degreesToRadians(-5.9), 0),
                ),
            ),
        ]

        self.layout = AprilTagFieldLayout(
            # config.code_path
            # + "reefscape-apriltags.json"
            config.code_path
            + "firehouse-apriltags.json"
        )

        scheduler.registerSubsystem(self)

    def periodic(self):
        pass

    def test(self):
        # result = self.camera.getLatestResult()
        # for target in result.getTargets():
        #     target.getPitch()
        # print(result.getTargets())
        pass

    def cur_atag(
        self, cam_i: int, red_id: int, blue_id: int
    ) -> Tuple[float, float] | None:
        atag_id = red_id if config.is_red() else blue_id
        result = self.cameras[cam_i].getLatestResult()
        for target in result.getTargets():
            if target.fiducialId == atag_id:
                return (
                    units.degreesToRadians(target.getPitch()),
                    units.degreesToRadians(target.getYaw()),
                )
        return None

    def estimate_multitag_pose(self, robot_angle: float) -> List[Tuple[Pose2d, float]]:
        tag_info = []
        for cam, transform in self.cameras:
            res = cam.getLatestResult()
            tags = res.getTargets()
            for tag in tags:
                tag_info.append(
                    (
                        tag.getFiducialId(),
                        units.degreesToRadians(-tag.getYaw()),
                        transform.translation()
                        .toTranslation2d()
                        .rotateBy(Rotation2d(robot_angle)),
                    )
                )
        poses = []
        for i in range(len(tag_info)):
            for j in range(i + 1, len(tag_info)):
                poses.append(
                    self.estimate_2tag_pose(robot_angle, tag_info[i], tag_info[j])
                )
        return poses

    def estimate_2tag_pose(
        self,
        robot_angle: float,
        # ID, yaw angle, camera offset from robot origin
        tag1: Tuple[int, float, Translation2d],
        tag2: Tuple[int, float, Translation2d],
    ) -> Tuple[Pose2d, float]:
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
            Pose2d(x, y, robot_angle) + Transform2d(tag1[2], Rotation2d()),
            confidence,
        )
