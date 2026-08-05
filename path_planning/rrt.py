import math
import random
from typing import Tuple, List


def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def steer(from_pt, to_pt, step_size):
    dx = to_pt[0] - from_pt[0]
    dy = to_pt[1] - from_pt[1]
    d = math.hypot(dx, dy)
    if d <= step_size:
        return to_pt
    return (from_pt[0] + dx / d * step_size, from_pt[1] + dy / d * step_size)


def collision_free(grid, a: Tuple[float, float], b: Tuple[float, float]) -> bool:
    # discretize line and check grid cells
    steps = int(math.ceil(distance(a, b) * 2))
    for i in range(steps + 1):
        t = i / max(1, steps)
        x = int(round(a[0] + (b[0] - a[0]) * t))
        y = int(round(a[1] + (b[1] - a[1]) * t))
        if not grid.is_free(x, y):
            return False
    return True


def rrt(grid, start: Tuple[int, int], goal: Tuple[int, int], max_iters: int = 2000, step_size: float = 5.0, goal_sample_rate: float = 0.05):
    # nodes: list of points (float x,y)
    nodes: List[Tuple[float, float]] = [start]
    parent = {0: None}
    visited_order = []

    for it in range(max_iters):
        if random.random() < goal_sample_rate:
            sample = goal
        else:
            sample = (random.uniform(0, grid.width - 1), random.uniform(0, grid.height - 1))

        # find nearest
        nearest_i = min(range(len(nodes)), key=lambda i: distance(nodes[i], sample))
        new_pt = steer(nodes[nearest_i], sample, step_size)
        if not grid.in_bounds(int(round(new_pt[0])), int(round(new_pt[1]))):
            continue
        if not collision_free(grid, nodes[nearest_i], new_pt):
            continue

        nodes.append(new_pt)
        parent[len(nodes) - 1] = nearest_i
        visited_order.append(new_pt)

        if distance(new_pt, goal) <= step_size and collision_free(grid, new_pt, goal):
            # found
            nodes.append(goal)
            parent[len(nodes) - 1] = len(nodes) - 2
            # reconstruct path
            path = [len(nodes) - 1]
            while parent[path[-1]] is not None:
                path.append(parent[path[-1]])
            path_pts = [nodes[i] for i in reversed(path)]
            return path_pts, nodes, parent, visited_order

    return None, nodes, parent, visited_order
