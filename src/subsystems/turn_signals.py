import time

from commands2 import CommandScheduler, Subsystem
from wpilib import Relay

from subsystems.manipulator_controller import ManipulatorController
from subsystems.claw import Claw
from utils import time_f


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

        self.relay = Relay(0, Relay.Direction.kBothDirections)
        self.flash_side = 0
        self.should_turn_signal = True

    @time_f("periodic turn signals")
    def periodic(self):
        if self.manip_controller.stalk_selection is not None and self.claw.has_coral():
            self.flash_side = -1 if self.manip_controller.stalk_selection % 2 else 1
        else:
            self.flash_side = 0

        now = time.time()
        if self.should_turn_signal:
            if now % 0.5 < 0.25:
                self.signal(self.flash_side, False)
            else:
                self.signal(self.flash_side, True)

    def signal(self, side: int, state: bool):
        if side == 1:
            self.relay.set(Relay.Value.kForward if state else Relay.Value.kOff)
        elif side == -1:
            self.relay.set(Relay.Value.kReverse if state else Relay.Value.kOff)
        elif side == 2:
            self.relay.set(Relay.Value.kOn if state else Relay.Value.kOff)
        else:
            self.relay.set(Relay.Value.kOff)
