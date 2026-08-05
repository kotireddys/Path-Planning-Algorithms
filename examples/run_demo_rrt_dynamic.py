import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from path_planning.rrt_dynamic import rrt_dynamic
from path_planning.dynamic_obstacle import DynamicObstacle

def run_dynamic_demo():
    print("Running 3D Dynamic RRT Demo...")
    
    bounds = (20.0, 20.0, 20.0)
    
    # Obstacles moving in various directions
    obstacles = [
        DynamicObstacle(pos=(10, 5, 5), vel=(0, 1, 0), size=2),
        DynamicObstacle(pos=(5, 15, 10), vel=(1, -1, 0), size=2),
        DynamicObstacle(pos=(15, 10, 15), vel=(-1, 0, -1), size=2)
    ]
    
    start = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0) # x, y, z, yaw, pitch, time
    goal = (19.0, 19.0, 19.0)
    
    path = rrt_dynamic(start, goal, bounds, obstacles, velocity=2.0)
    
    if path:
        print(f"Success! Path found with {len(path)} waypoints.")
        for i, pt in enumerate(path):
            print(f"Step {i}: {pt}")
    else:
        print("No path found.")

if __name__ == "__main__":
    run_dynamic_demo()
