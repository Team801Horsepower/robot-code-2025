from wpilib.interfaces import GenericHID
from commands2 import Subsystem, CommandScheduler

class ManipulatorController(Subsystem):
    def __init__(self, scheduler: CommandScheduler, controller_id: int = 1):
        scheduler.registerSubsystem(self)

        self.controller = GenericHID(controller_id)
        self.button_states = [None for _ in range(32)]
        self.joystick_states = [None for _ in range(32)]
        self.branch_selection = None

    def get_button_states(self):
        self.button_states = [self.controller.getRawButton(i + 1) for i in range(32)] # one-indexed for some reason

    def get_joystick_states(self):
        self.joystick_states = [self.controller.getRawAxis(i) for i in range(6)]

    def periodic(self):
        self.get_button_states()
        self.get_joystick_states()

    def get_branch_selection(self):
        def axis_to_selection(value):
            return round(6 * (value + 1)) % 12

        self.branch_selection = axis_to_selection(self.joystick_states[0])
        
