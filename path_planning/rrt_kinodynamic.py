import math
import random
from typing import Tuple, List, Optional

# Define an obstacle as (x, y, z, size) for a cube
Obstacle = Tuple[float, float, float, float]
# State: (x, y, z, yaw, pitch)
State = Tuple[float, float, float, float, float]

def distance_3d(a: State, b: State) -> float:
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)

def steer_kinodynamic(from_state: State, to_point: Tuple[float, float, float], step_size: float, max_yaw_change: float, max_pitch_change: float) -> State:
    x, y, z, yaw, pitch = from_state
    tx, ty, tz = to_point
    
    # Calculate desired yaw and pitch
    dx, dy, dz = tx - x, ty - y, tz - z
    
    target_yaw = math.atan2(dy, dx)
    target_pitch = math.atan2(-dz, math.sqrt(dx**2 + dy**2))
    
    # Normalize angle differences
    def normalize_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    dyaw = normalize_angle(target_yaw - yaw)
    dpitch = normalize_angle(target_pitch - pitch)
    
    # Limit changes
    dyaw = max(-max_yaw_change, min(max_yaw_change, dyaw))
    dpitch = max(-max_pitch_change, min(max_pitch_change, dpitch))
    
    new_yaw = yaw + dyaw
    new_pitch = pitch + dpitch
    
    # Calculate new position
    new_x = x + math.cos(new_pitch) * math.cos(new_yaw) * step_size
    new_y = y + math.cos(new_pitch) * math.sin(new_yaw) * step_size
    new_z = z - math.sin(new_pitch) * step_size
    
    return (new_x, new_y, new_z, new_yaw, new_pitch)

def collision_free_3d(a: State, b: State, obstacles: List[Obstacle]) -> bool:
    # Discretize segment for collision checking
    d = distance_3d(a, b)
    steps = int(math.ceil(d * 5))
    for i in range(steps + 1):
        t = i / max(1, steps)
        px = a[0] + (b[0] - a[0]) * t
        py = a[1] + (b[1] - a[1]) * t
        pz = a[2] + (b[2] - a[2]) * t
        
        for obs in obstacles:
            ox, oy, oz, size = obs
            if (ox <= px <= ox + size) and (oy <= py <= oy + size) and (oz <= pz <= oz + size):
                return False
    return True

def rrt_3d_kinodynamic(
    start: State, 
    goal: Tuple[float, float, float], 
    bounds: Tuple[float, float, float], 
    obstacles: List[Obstacle], 
    max_iters: int = 5000, 
    step_size: float = 1.0, 
    goal_sample_rate: float = 0.05,
    max_yaw_change: float = math.pi/4,
    max_pitch_change: float = math.pi/4
) -> Optional[List[State]]:
    
    nodes: List[State] = [start]
    parent = {0: None}
    
    for _ in range(max_iters):
        if random.random() < goal_sample_rate:
            sample = goal
        else:
            sample = (random.uniform(0, bounds[0]), random.uniform(0, bounds[1]), random.uniform(0, bounds[2]))
            
        nearest_i = min(range(len(nodes)), key=lambda i: distance_3d(nodes[i], (sample[0], sample[1], sample[2])))
        new_state = steer_kinodynamic(nodes[nearest_i], sample, step_size, max_yaw_change, max_pitch_change)
        
        # Check bounds
        if not (0 <= new_state[0] <= bounds[0] and 0 <= new_state[1] <= bounds[1] and 0 <= new_state[2] <= bounds[2]):
            continue
            
        if not collision_free_3d(nodes[nearest_i], new_state, obstacles):
            continue
            
        nodes.append(new_state)
        parent[len(nodes) - 1] = nearest_i
        
        # Goal check
        if distance_3d(new_state, (goal[0], goal[1], goal[2], 0, 0)) <= step_size and collision_free_3d(new_state, (goal[0], goal[1], goal[2], 0, 0), obstacles):
            # For simplicity, we assume goal is reached if we get close
            path = [len(nodes) - 1]
            while parent[path[-1]] is not None:
                path.append(parent[path[-1]])
            return [nodes[i] for i in reversed(path)]
            
    return None
