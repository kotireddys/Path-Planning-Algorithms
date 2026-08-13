import math
import random
from typing import Tuple, List, Dict

from .rrt import distance, steer, collision_free


def _sample_unit_disk():
    while True:
        x = random.uniform(-1, 1)
        y = random.uniform(-1, 1)
        if x * x + y * y <= 1.0:
            return x, y


def _informed_sample(grid, start, goal, c_max):
    """Uniform sample inside the ellipse with foci start/goal whose major
    axis is c_max — every point inside it could still improve on a path
    that already costs c_max, so sampling outside is provably wasted once
    a solution exists. Falls back to uniform sampling before any solution
    is found (c_max is None) or if the ellipse is degenerate."""
    c_min = distance(start, goal)
    if c_max is None or c_max == float('inf') or c_max <= c_min:
        return (random.uniform(0, grid.width - 1), random.uniform(0, grid.height - 1))

    center = ((start[0] + goal[0]) / 2.0, (start[1] + goal[1]) / 2.0)
    theta = math.atan2(goal[1] - start[1], goal[0] - start[0])
    r1 = c_max / 2.0
    r2 = math.sqrt(max(c_max ** 2 - c_min ** 2, 0.0)) / 2.0

    bx, by = _sample_unit_disk()
    ex, ey = r1 * bx, r2 * by
    x = center[0] + ex * math.cos(theta) - ey * math.sin(theta)
    y = center[1] + ex * math.sin(theta) + ey * math.cos(theta)
    return (x, y)


def informed_rrt_star(grid, start: Tuple[int, int], goal: Tuple[int, int], max_iters: int = 3000,
                       step_size: float = 5.0, goal_sample_rate: float = 0.05, neighbor_radius: float = 10.0):
    """Same tree growth, parent selection, and rewiring as rrt_star(), but
    once a first solution is found, sampling is restricted to the
    prolate-hyperspheroid (an ellipse in 2D) that could still contain a
    shorter path (Gammell et al., 2014). Unlike rrt_star(), this keeps
    sampling for the full max_iters budget even after finding a solution,
    since informed sampling only pays off by refining that solution further.

    Returns (path_pts, nodes, parent, visited_order) — same shape as
    rrt_star(), where path_pts is the best path found by the time the
    budget runs out (None if no solution was ever found).
    """
    nodes: List[Tuple[float, float]] = [start]
    parent: Dict[int, int] = {0: None}
    cost: Dict[int, float] = {0: 0.0}
    visited_order: List[Tuple[float, float]] = []

    best_goal_idx = None
    best_cost = float('inf')

    def near_indices(pt):
        return [i for i, n in enumerate(nodes) if distance(n, pt) <= neighbor_radius]

    for it in range(max_iters):
        if random.random() < goal_sample_rate:
            sample = goal
        else:
            sx, sy = _informed_sample(grid, start, goal, best_cost)
            sample = (min(max(sx, 0), grid.width - 1), min(max(sy, 0), grid.height - 1))

        nearest_i = min(range(len(nodes)), key=lambda i: distance(nodes[i], sample))
        new_pt = steer(nodes[nearest_i], sample, step_size)
        xi, yi = int(round(new_pt[0])), int(round(new_pt[1]))
        if not grid.in_bounds(xi, yi):
            continue
        if not collision_free(grid, nodes[nearest_i], new_pt):
            continue

        neighbors = near_indices(new_pt)
        best_parent = nearest_i
        best_new_cost = cost[nearest_i] + distance(nodes[nearest_i], new_pt)
        for n_i in neighbors:
            if collision_free(grid, nodes[n_i], new_pt):
                c = cost[n_i] + distance(nodes[n_i], new_pt)
                if c < best_new_cost:
                    best_new_cost = c
                    best_parent = n_i

        nodes.append(new_pt)
        idx = len(nodes) - 1
        parent[idx] = best_parent
        cost[idx] = best_new_cost
        visited_order.append(new_pt)

        for n_i in neighbors:
            if n_i == idx:
                continue
            if collision_free(grid, new_pt, nodes[n_i]):
                new_cost = cost[idx] + distance(nodes[idx], nodes[n_i])
                if new_cost < cost.get(n_i, float('inf')):
                    parent[n_i] = idx
                    cost[n_i] = new_cost

        if distance(new_pt, goal) <= step_size and collision_free(grid, new_pt, goal):
            reach_cost = cost[idx] + distance(new_pt, goal)
            if reach_cost < best_cost:
                if best_goal_idx is None:
                    nodes.append(goal)
                    best_goal_idx = len(nodes) - 1
                parent[best_goal_idx] = idx
                cost[best_goal_idx] = reach_cost
                best_cost = reach_cost

    if best_goal_idx is None:
        return None, nodes, parent, visited_order

    path = [best_goal_idx]
    while parent[path[-1]] is not None:
        path.append(parent[path[-1]])
    path_pts = [nodes[i] for i in reversed(path)]
    return path_pts, nodes, parent, visited_order
