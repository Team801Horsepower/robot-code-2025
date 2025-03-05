from commands2 import CommandScheduler, Subsystem
from wpimath.geometry import Translation2d, Transform3d, Rotation3d, Translation3d
from typing import Tuple, List, Optional
from photonlibpy.photonCamera import PhotonCamera
from wpimath import units

import config


class Vision(Subsystem):
    def __init__(self, scheduler: CommandScheduler):
        scheduler.registerSubsystem(self)

        self.cameras: List[Tuple[PhotonCamera, Transform3d]] = [
            (
                PhotonCamera("FrontLeft"),
                Transform3d(
                    # Translation3d(0.60880625, 0.29845, 0.428625),
                    # Translation3d(0, 0, 0.428625),
                    Translation3d(-0.11146, 0.23460, 0.27491),
                    Rotation3d(
                        0, units.degreesToRadians(20), units.degreesToRadians(340)
                    ),
                ),
            ),
            (
                PhotonCamera("FrontRight"),
                Transform3d(
                    Translation3d(-0.11146, -0.23460, 0.428625),
                    Rotation3d(
                        0, units.degreesToRadians(20), units.degreesToRadians(20)
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
            # (
            #     PhotonCamera("BackRight"),
            #     Transform3d(
            #         Translation3d(-0.16406, -0.25998, 0.277),
            #         Rotation3d(0, units.degreesToRadians(25), units.degreesToRadians(210)),
            #     ),
            # ),
        ]
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

    # (ntags, (position, confidence, deviation))
    def pos_report(
        self, robot_angle: float
    ) -> Tuple[int, Optional[Tuple[Translation2d, float, float]]]:
        return (0, None)

    # returns corrected heading based on april tags
    def heading_correction(self, robot_angle: float) -> Optional[float]:
        return None
