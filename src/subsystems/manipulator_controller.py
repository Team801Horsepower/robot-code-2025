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
        
        self.left_player_station = False #LPS
        self.right_player_station = False #RPS
        self.left_processor = False #LP
        self.right_processor = False #RP
        self.power = False #P
        self.reset = False #R
        self.deep_ascent = False #DA
        self.misc1 = False #M1
        self.misc2 = False #M2
        self.misc3 = False #M3
        self.misc4 = False #M4

# Diagram, for dummies and smart people alike - Hudson

# ___________________________
#|                           |\
#|   T   M1         ______   | \
#|   T   M2        / LPS  \  |  |
#|   T   M3       /LP    RP\ |  |
#|   T   M4       \  RPS   / | R|
#|       DA        \______/  |  |
#|___________________________| P|
#\                            \ |
# \_____________________________|

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

#ok so we have the buttons returning values
#What will we do with these values
    #Climb
    #Coral placement
        #DriveToPose
        #Elevator arm positioning
        #run a check?
        #Release coral
    #Algae placement
        #Need elevator arm extend independent button (full height)
        #Need algae release button independent
    #Coral collection
        #Any auto straightening?
        #Run gatherer
        #Check if gathered
        #report to manipulator
    #Algae collection
        #DriveToPose and appropriate elevator positioning
        #run a check?
        #Gather algae
        #Algae collection vs coral placement
#What about when the inputs overlap
    #Have a drivetopose cancel
    #Coral placement over all else
        #Coral placement will not trigger unless coral gathered
    #Then whatever else, dunno the subsystems
#Commmunicate w/ manipulator for when some object is gathered/dispensed? How?
    #Drive controller vibrate?
    #LEDs?
#What if the robot gets bumped into while dispensing coral or autopositioning
    #Periodically run position check, run coral gather IMMEDIATELY if outside of some zone
    #idk