import math
import random
import numpy as np
from typing import Tuple, List, Optional
from path_planning.dynamic_obstacle import DynamicObstacle

def distance_3d(a, b):
    return math.sqrt(sum((a[i] - b[i])**2 for i in range(3)))

def steer_dynamic(from_state, to_point, step_size, velocity):
    # from_state: (x, y, z, yaw, pitch, t)
    # to_point: (x, y, z)
    # Returns: (x, y, z, yaw, pitch, t_new)
    
    fx, fy, fz, fyaw, fpitch, ft = from_state
    
    # Calculate direction to target
    dx = to_point[0] - fx
    dy = to_point[1] - fy
    dz = to_point[2] - fz
    dist = math.sqrt(dx**2 + dy**2 + dz**2)
    
    if dist <= step_size:
        move_dist = dist
        new_pt = to_point
    else:
        move_dist = step_size
        ratio = step_size / dist
        new_pt = (fx + dx * ratio, fy + dy * ratio, fz + dz * ratio)
        
    # Estimate new time
    dt = move_dist / velocity
    new_t = ft + dt
    
    # Simple steering constraint (can be complex, keeping simple for demo)
    new_yaw = math.atan2(dy, dx)
    new_pitch = math.atan2(dz, math.sqrt(dx**2 + dy**2))
    
    return (new_pt[0], new_pt[1], new_pt[2], new_yaw, new_pitch, new_t)

def collision_free_dynamic(a, b, t_a, t_b, obstacles: List[DynamicObstacle]):
    # Discretize segment
    steps = 5
    for i in range(steps + 1):
        ratio = i / steps
        t = t_a + (t_b - t_a) * ratio
        point = (a[0] + (b[0] - a[0]) * ratio,
                 a[1] + (b[1] - a[1]) * ratio,
                 a[2] + (b[2] - a[2]) * ratio)
        
        for obs in obstacles:
            if obs.is_colliding(point, t):
                return False
    return True

def rrt_dynamic(start, goal, bounds, obstacles, velocity=1.0, max_iters=5000, step_size=1.0):
    # state: (x, y, z, yaw, pitch, t)
    nodes = [start]
    parent = {0: None}
    
    for _ in range(max_iters):
        # Sample space (including goal bias)
        if random.random() < 0.1:
            sample = goal
        else:
            sample = (random.uniform(0, bounds[0]), random.uniform(0, bounds[1]), random.uniform(0, bounds[2]))
            
        # Find nearest
        nearest_i = min(range(len(nodes)), key=lambda i: distance_3d(nodes[i][:3], sample))
        
        # Steer
        new_state = steer_dynamic(nodes[nearest_i], sample, step_size, velocity)
        
        # Check bounds
        if not all(0 <= new_state[i] <= bounds[i] for i in range(3)):
            continue
            
        # Check collision
        if not collision_free_dynamic(nodes[nearest_i][:3], new_state[:3], nodes[nearest_i][5], new_state[5], obstacles):
            continue
            
        nodes.append(new_state)
        parent[len(nodes) - 1] = nearest_i
        
        if distance_3d(new_state[:3], goal) <= step_size:
            path = [len(nodes) - 1]
            while parent[path[-1]] is not None:
                path.append(parent[path[-1]])
            return [nodes[i] for i in reversed(path)]
            
    return None
