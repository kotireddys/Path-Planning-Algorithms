import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from path_planning.rrt_kinodynamic import rrt_3d_kinodynamic
import math

def run_demo():
    start = (0.0, 0.0, 0.0, 0.0, 0.0) # x, y, z, yaw, pitch
    goal = (10.0, 10.0, 10.0)
    bounds = (15.0, 15.0, 15.0)
    obstacles = [(5.0, 5.0, 0.0, 2.0)] # x, y, z, size
    
    path = rrt_3d_kinodynamic(
        start, 
        goal, 
        bounds, 
        obstacles, 
        max_iters=10000, 
        step_size=0.5,
        max_yaw_change=math.pi/8,
        max_pitch_change=math.pi/8
    )
    
    if path:
        print("Path found!")
        for i, state in enumerate(path):
            print(f"Step {i}: {state}")
    else:
        print("Path not found.")

if __name__ == "__main__":
    run_demo()
