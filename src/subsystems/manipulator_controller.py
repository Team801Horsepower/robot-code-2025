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

    def update_button_states(self):
        self.button_states = [self.controller.getRawButtonPressed(i + 1) for i in range(32)]

    def update_joystick_states(self):
        self.joystick_states = [self.controller.getRawAxis(i) for i in range(6)]

    def periodic(self):
        self.update_button_states()
        self.update_joystick_states()
        self.update_height_selection()
        self.update_tree_selection()

    def update_tree_selection(self):
        def axis_to_selection(value):
            return round(6 * (value + 1)) % 12

        self.tree_selection = axis_to_selection(self.joystick_states[0])

    def update_height_selection(self):
        for i in range(4):
            if self.button_states[i] is True:  
                self.height_selection = i + 1  

    # --- Friendly Accessors ---
    
    def get_button_states(self):
        """Returns the current button states as a list of booleans."""
        return self.button_states

    def get_joystick_states(self):
        """Returns the current joystick axis states as a list of floats."""
        return self.joystick_states

    @property
    def tree(self):
        """Returns the selected tree value."""
        return self.tree_selection

    @property
    def height(self):
        """Returns the selected height value."""
        return self.height_selection

    def __str__(self):
        """String representation for debugging."""
        return (
            f"ManipulatorController(\n"
            f"  tree_selection={self.tree_selection},\n"
            f"  height_selection={self.height_selection},\n"
            f"  button_states={self.button_states},\n"
            f"  joystick_states={self.joystick_states}\n"
            f")"
        )

