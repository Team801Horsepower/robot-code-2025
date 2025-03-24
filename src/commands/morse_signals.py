from commands2 import Command
import time

from subsystems.turn_signals import TurnSignals


class PlaceCoral(Command):
    def __init__(self, turn_signals: TurnSignals):
        self.turn_signals = TurnSignals
        self.finished = False

    def initialize(self):
        pass

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.finished
