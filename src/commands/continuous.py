from commands2 import Command


class Continuous(Command):
    def __init__(self, inner: Command):
        self.inner = inner

    def initialize(self):
        self.inner.initialize()

    def execute(self):
        self.inner.execute()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self.inner.end(interrupted)

    def inner_finished(self) -> bool:
        return self.inner.isFinished()
