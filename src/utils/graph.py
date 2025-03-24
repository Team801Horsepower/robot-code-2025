import json
from typing import List
from astar import AStar
from wpimath.geometry import Translation2d

from config import flip_red


class Graph(AStar):
    def __init__(self, file_path: str):
        with open(file_path, "r") as f:
            data = json.load(f)
        # self.nodes = [Translation2d(flip_red(l[0]), l[1]) for l in data["nodes"]]
        self.nodes = [flip_red(Translation2d(l[0], l[1])) for l in data["nodes"]]
        self.edges = [(l[0], l[1]) for l in data["edges"]]
        # self.shoot_idxs = data["shoot_idxs"]

    def neighbors(self, node: int) -> List[int]:
        n_li = []
        for start, end in self.edges:
            if start == node:
                n_li.append(end)
            if end == node:
                n_li.append(start)
        return n_li

    def distance_between(self, n1: int, n2: int) -> float:
        return (self.nodes[n1] - self.nodes[n2]).norm()

    def heuristic_cost_estimate(self, current: int, goal: int) -> float:
        return self.distance_between(current, goal)

    def create_path(
        self, start: Translation2d, end: Translation2d
    ) -> List[Translation2d]:
        def add_node(pos: Translation2d):
            closest_i, _ = min(
                enumerate(self.nodes), key=lambda tup: (tup[1] - pos).norm()
            )
            new_i = len(self.nodes)
            self.nodes.append(pos)
            self.edges.append((closest_i, new_i))

        start_i = len(self.nodes)
        add_node(start)
        add_node(end)

        path_idxs = self.astar(start_i, start_i + 1)
        if path_idxs is None:
            return []
        path = [self.nodes[i] for i in path_idxs]
        for _ in range(2):
            self.nodes.pop()
            self.edges.pop()

        return path
