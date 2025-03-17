import time

from commands2 import CommandScheduler, Subsystem
from wpilib import PowerDistribution, PWMMotorController

from subsystems.manipulator_controller import ManipulatorController
from subsystems.claw import Claw


class TurnSignals(Subsystem):
    def __init__(
        self,
        scheduler: CommandScheduler,
        manip_controller: ManipulatorController,
        claw: Claw,
    ):
        scheduler.registerSubsystem(self)

        self.claw = claw
        self.manip_controller = manip_controller

        self.pdp = PowerDistribution()
        self.spark = PWMMotorController("left_signal", 9)

        self.flash_side = 0

    def periodic(self):
        if self.manip_controller.stalk_selection is not None and self.claw.has_coral():
            self.flash_side = -1 if self.manip_controller.stalk_selection % 2 else 1
        else:
            self.flash_side = 0

        now = time.time()
        if now % 2 < 1:
            self.signal(self.flash_side, False)
        else:
            self.signal(self.flash_side, True)

    def signal(self, side: int, state: bool):
        if side == 1:
            self.spark.set(1 if state else 0)
            if state:
                self.signal(-1, False)
        elif side == -1:
            self.pdp.setSwitchableChannel(state)
            if state:
                self.signal(1, False)
        elif side == 2:
            self.signal(1, state)
            self.signal(-1, state)
        else:
            self.signal(1, False)
            self.signal(-1, False)
