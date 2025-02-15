from wpilib.interfaces import GenericHID
from commands2 import Subsystem, CommandScheduler

class ManipulatorController(Subsystem):
    def __init__(self, scheduler: CommandScheduler, controller_id: int = 1):
        scheduler.registerSubsystem(self)

        self.controller = GenericHID(controller_id)
        self.button_states = [None for _ in range(32)]
        self.joystick_states = [None for _ in range(6)]
        self.tree_selection = None
        self.height_selection = None

    def get_button_states(self):
        self.button_states = [self.controller.getRawButtonPressed(i + 1) for i in range(32)] # one-indexed for some reason

    def get_joystick_states(self):
        self.joystick_states = [self.controller.getRawAxis(i) for i in range(6)]

    def periodic(self):
        self.get_button_states()
        self.get_joystick_states()
        self.get_height_selection()

    def get_tree_selection(self):
        def axis_to_selection(value):
            return round(6 * (value + 1)) % 12

        self.tree_selection = axis_to_selection(self.joystick_states[0])

    def get_height_selection(self):
        for i in range(4):
            if self.button_states[i]:
                self.button_selection = i + 1 # the manual's naming convention starts at L1, so we'll stick with that
