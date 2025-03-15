from wpilib.interfaces import GenericHID
from commands2 import Subsystem, CommandScheduler

import config


class ManipulatorController(Subsystem):
    def __init__(self, scheduler: CommandScheduler, controller_id: int = 1):
        scheduler.registerSubsystem(self)

        self.controller = GenericHID(controller_id)

        # TODO: Add actual values of the inputs
        self.input_indices = {
            "branch": [1, 2, 3, 4],
            "misc": [0, 0],
            "algae": 8,
            "g_pickup": 7,
            "processor": 6,
            "barge": 5,
            "climb": 9,
        }

        self.stalk_selection = None

        self.arm_setpoint = config.transit_setpoint

    def periodic(self):
        self.update_tree_selection()
        if (
            self.controller.getRawButtonPressed(self.input_indices["algae"])
            and self.stalk_selection is not None
        ):
            self.arm_setpoint = config.algae_reef_setpoints[
                int(self.stalk_selection / 2) % 2
            ]
        elif self.controller.getRawButtonPressed(self.input_indices["g_pickup"]):
            self.arm_setpoint = config.ground_pickup_setpoint
        elif self.controller.getRawButtonPressed(self.input_indices["processor"]):
            self.arm_setpoint = config.processor_setpoint
        elif self.controller.getRawButtonPressed(self.input_indices["barge"]):
            self.arm_setpoint = config.barge_setpoint
        else:
            for i, index in enumerate(self.input_indices["branch"]):
                if self.controller.getRawButtonPressed(index):
                    self.arm_setpoint = config.reef_setpoints[i]
                    break

    def update_tree_selection(self):
        def axis_to_selection(value):
            return (7 - round(6 * (value + 1))) % 12

        self.stalk_selection = axis_to_selection(self.controller.getRawAxis(0))
