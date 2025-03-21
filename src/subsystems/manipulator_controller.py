from wpilib import SmartDashboard
from wpilib.interfaces import GenericHID
from commands2 import Subsystem, CommandScheduler

import config
from utils import time_f


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

        self.target_level = None

        self.reef_algae_selected = False

        self.disable_pathfinding = False

    @time_f("periodic manip controller")
    def periodic(self):
        if (
            self.arm_setpoint == config.algae_reef_setpoints[0]
            or self.arm_setpoint == config.algae_reef_setpoints[1]
        ) and self.stalk_selection is not None:
            self.arm_setpoint = config.algae_reef_setpoints[
                int(self.stalk_selection / 2) % 2
            ]
        if (
            self.controller.getRawButtonPressed(self.input_indices["algae"])
            and self.stalk_selection is not None
        ):
            self.arm_setpoint = config.algae_reef_setpoints[
                int(self.stalk_selection / 2) % 2
            ]
            SmartDashboard.putString("setpoint selection", "ALGAE")
            self.target_level = None
            self.reef_algae_selected = True
        elif self.controller.getRawButtonPressed(self.input_indices["g_pickup"]):
            self.arm_setpoint = config.ground_pickup_setpoint
            SmartDashboard.putString("setpoint selection", "GROUND ALGAE")
            self.target_level = None
            self.reef_algae_selected = False
        elif self.controller.getRawButtonPressed(self.input_indices["processor"]):
            self.arm_setpoint = config.processor_setpoint
            SmartDashboard.putString("setpoint selection", "PROCESSOR")
            self.target_level = None
            self.reef_algae_selected = False
        elif self.controller.getRawButtonPressed(self.input_indices["barge"]):
            self.arm_setpoint = config.barge_setpoint
            SmartDashboard.putString("setpoint selection", "BARGE")
            self.target_level = None
            self.reef_algae_selected = False
        else:
            for i, index in enumerate(self.input_indices["branch"]):
                if self.controller.getRawButtonPressed(index):
                    self.arm_setpoint = config.reef_setpoints[i]
                    self.target_level = i
                    self.sb_reef_sel()
                    self.reef_algae_selected = False
                    break

        self.update_tree_selection()

        self.disable_pathfinding = self.controller.getRawButton(
            self.input_indices["climb"]
        )

    def update_tree_selection(self):
        def axis_to_selection(value):
            return (7 - round(6 * (value + 1))) % 12

        self.stalk_selection = axis_to_selection(self.controller.getRawAxis(0))

        for i in range(6):
            SmartDashboard.putBoolean(
                f"reef selection {i}", self.stalk_selection // 2 == i
            )

        if self.target_level is not None:
            self.sb_reef_sel()

    def sb_reef_sel(self):
        if self.stalk_selection is None or self.target_level is None:
            return
        letter = "HGFEDCBALKJI"[self.stalk_selection]
        number = str(self.target_level + 1)
        SmartDashboard.putString("setpoint selection", letter + number)
