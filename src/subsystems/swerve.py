from rev import SparkMax, SparkFlex
from wpimath.geometry import Rotation2d
from phoenix6.hardware.cancoder import CANcoder

import config


class Swerve:
    def __init__(
        self,
        drive: SparkFlex,
        turn: SparkMax,
        turn_cancoder: CANcoder,
        cancoder_offset: float,
    ):
        self.drive_motor = drive
        self.drive_pid = self.drive_motor.getClosedLoopController()
        self.drive_encoder = self.drive_motor.getEncoder()

        self.turn_motor = turn
        self.turn_pid = self.turn_motor.getClosedLoopController()
        self.turn_encoder = self.turn_motor.getEncoder()

        self.turn_cancoder = turn_cancoder
        self.cancoder_offset = cancoder_offset
        self.reset_from_abs_enc()

        self.update_prevs()

    def reset_from_abs_enc(self):
        cur_turn = (
            self.turn_cancoder.get_absolute_position().value - self.cancoder_offset
        )
        if cur_turn < -0.5:
            cur_turn += 1
        if cur_turn > 0.5:
            cur_turn -= 1
        conv_factor = (
            self.turn_motor.configAccessor.encoder.getPositionConversionFactor()
        )
        self.turn_encoder.setPosition(cur_turn * config.turn_gear_ratio * conv_factor)

    def rotation(self) -> Rotation2d:
        return Rotation2d(self.turn_encoder.getPosition())

    def update_prevs(self):
        self.prev_drive_enc = self.drive_encoder.getPosition()
        self.prev_rotation = self.rotation()
