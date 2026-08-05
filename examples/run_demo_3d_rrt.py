import os
import random
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from path_planning.rrt_3d_continuous import rrt_3d

def run_unstructured_demo():
    print("Running 3D Continuous RRT (Unstructured - Cubes) Demo...")
    
    # Define bounds (20x20x20)
    bounds = (20.0, 20.0, 20.0)
    
    # Generate random obstacles (x, y, z, size)
    obstacles = []
    for _ in range(20):
        x = random.uniform(2, 18)
        y = random.uniform(2, 18)
        z = random.uniform(2, 18)
        size = random.uniform(1, 3)
        obstacles.append((x, y, z, size))
    
    start = (0.0, 0.0, 0.0)
    goal = (19.0, 19.0, 19.0)
    
    path = rrt_3d(start, goal, bounds, obstacles, max_iters=10000, step_size=1.0)
    
    if path:
        print(f"Success! Path found with {len(path)} waypoints.")
    else:
        print("No path found.")

if __name__ == "__main__":
    run_unstructured_demo()
