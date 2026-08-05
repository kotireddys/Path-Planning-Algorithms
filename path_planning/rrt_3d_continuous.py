import math
import random
from typing import Tuple, List

# Define an obstacle as (x, y, z, size) for a cube
Obstacle = Tuple[float, float, float, float]

def distance_3d(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt(sum((a[i] - b[i])**2 for i in range(3)))

def steer_3d(from_pt, to_pt, step_size):
    d = distance_3d(from_pt, to_pt)
    if d <= step_size:
        return to_pt
    ratio = step_size / d
    return tuple(from_pt[i] + (to_pt[i] - from_pt[i]) * ratio for i in range(3))

def collision_free_3d(a: Tuple[float, float, float], b: Tuple[float, float, float], obstacles: List[Obstacle]) -> bool:
    # Discretize segment into points for robust AABB collision checking
    steps = int(math.ceil(distance_3d(a, b) * 5)) # 5 points per unit length
    for i in range(steps + 1):
        t = i / max(1, steps)
        px = a[0] + (b[0] - a[0]) * t
        py = a[1] + (b[1] - a[1]) * t
        pz = a[2] + (b[2] - a[2]) * t
        
        for obs in obstacles:
            ox, oy, oz, size = obs
            # Check if point is inside cube
            if (ox <= px <= ox + size) and (oy <= py <= oy + size) and (oz <= pz <= oz + size):
                return False
    return True

def rrt_3d(start: Tuple[float, float, float], goal: Tuple[float, float, float], bounds: Tuple[float, float, float], obstacles: List[Obstacle], max_iters: int = 5000, step_size: float = 1.0, goal_sample_rate: float = 0.05):
    nodes: List[Tuple[float, float, float]] = [start]
    parent = {0: None}
    
    for _ in range(max_iters):
        if random.random() < goal_sample_rate:
            sample = goal
        else:
            sample = tuple(random.uniform(0, b) for b in bounds)
            
        nearest_i = min(range(len(nodes)), key=lambda i: distance_3d(nodes[i], sample))
        new_pt = steer_3d(nodes[nearest_i], sample, step_size)
        
        # Check bounds
        if not all(0 <= new_pt[i] <= bounds[i] for i in range(3)):
            continue
            
        if not collision_free_3d(nodes[nearest_i], new_pt, obstacles):
            continue
            
        nodes.append(new_pt)
        parent[len(nodes) - 1] = nearest_i
        
        if distance_3d(new_pt, goal) <= step_size and collision_free_3d(new_pt, goal, obstacles):
            nodes.append(goal)
            parent[len(nodes) - 1] = len(nodes) - 2
            path = [len(nodes) - 1]
            while parent[path[-1]] is not None:
                path.append(parent[path[-1]])
            return [nodes[i] for i in reversed(path)]
            
    return None
