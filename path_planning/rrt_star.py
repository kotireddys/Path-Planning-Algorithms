import math
import random
from typing import Tuple, List, Dict

from .rrt import distance, steer, collision_free


def rrt_star(grid, start: Tuple[int, int], goal: Tuple[int, int], max_iters: int = 2000, step_size: float = 5.0, goal_sample_rate: float = 0.05, neighbor_radius: float = 10.0):
    nodes: List[Tuple[float, float]] = [start]
    parent: Dict[int, int] = {0: None}
    cost: Dict[int, float] = {0: 0.0}
    visited_order: List[Tuple[float, float]] = []

    def near_indices(pt):
        return [i for i, n in enumerate(nodes) if distance(n, pt) <= neighbor_radius]

    for it in range(max_iters):
        if random.random() < goal_sample_rate:
            sample = goal
        else:
            sample = (random.uniform(0, grid.width - 1), random.uniform(0, grid.height - 1))

        nearest_i = min(range(len(nodes)), key=lambda i: distance(nodes[i], sample))
        new_pt = steer(nodes[nearest_i], sample, step_size)
        xi, yi = int(round(new_pt[0])), int(round(new_pt[1]))
        if not grid.in_bounds(xi, yi):
            continue
        if not collision_free(grid, nodes[nearest_i], new_pt):
            continue

        # choose parent among neighbors with lowest cost
        neighbors = near_indices(new_pt)
        best_parent = nearest_i
        best_cost = cost[nearest_i] + distance(nodes[nearest_i], new_pt)
        for n_i in neighbors:
            if collision_free(grid, nodes[n_i], new_pt):
                c = cost[n_i] + distance(nodes[n_i], new_pt)
                if c < best_cost:
                    best_cost = c
                    best_parent = n_i

        nodes.append(new_pt)
        idx = len(nodes) - 1
        parent[idx] = best_parent
        cost[idx] = best_cost
        visited_order.append(new_pt)

        # rewire neighbors
        for n_i in neighbors:
            if n_i == idx:
                continue
            if collision_free(grid, new_pt, nodes[n_i]):
                new_cost = cost[idx] + distance(nodes[idx], nodes[n_i])
                if new_cost < cost.get(n_i, float('inf')):
                    parent[n_i] = idx
                    cost[n_i] = new_cost

        # check goal reach
        if distance(new_pt, goal) <= step_size and collision_free(grid, new_pt, goal):
            nodes.append(goal)
            parent[len(nodes) - 1] = len(nodes) - 2
            # reconstruct path
            path = [len(nodes) - 1]
            while parent[path[-1]] is not None:
                path.append(parent[path[-1]])
            path_pts = [nodes[i] for i in reversed(path)]
            return path_pts, nodes, parent, visited_order

    return None, nodes, parent, visited_order
