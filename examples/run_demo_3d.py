import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from path_planning.grid3d import Grid3D
from path_planning.a_star_3d import astar_3d

def run_3d_demo():
    print("Running 3D A* Demo...")
    # Initialize a 20x20x20 3D grid with some obstacles
    grid = Grid3D(20, 20, 20, obstacle_prob=0.1, seed=42)
    start = (0, 0, 0)
    goal = (18, 18, 18)
    
    # Ensure start and goal are free
    grid.set_free(*start)
    grid.set_free(*goal)
    
    path, visited = astar_3d(grid, start, goal)
    
    if path:
        print(f"Success! Path found with {len(path)} steps.")
        print(f"Path: {path[:5]} ... {path[-5:]}")
    else:
        print("No path found.")

if __name__ == "__main__":
    run_3d_demo()
