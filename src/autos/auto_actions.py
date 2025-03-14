class GatherAction():
    def __init__(self, coral_station: int):
        self.coral_station = coral_station

        self.command = None

class ScorerAction():
    def __init__(self, stalk: int, branch: int):
        self.stalk = stalk
        self.branch = branch

        self.command = None
