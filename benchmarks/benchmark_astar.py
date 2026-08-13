import time
import timeit
from path_planning.grid import Grid
from path_planning.a_star import astar

def benchmark_astar():
    # Setup a grid
    grid_size = 500
    g = Grid(grid_size, grid_size, obstacle_prob=0.2, seed=42)
    start = (0, 0)
    goal = (grid_size-1, grid_size-1)

    # Time the A* execution
    start_time = time.perf_counter()
    path, visited = astar(g, start, goal)
    end_time = time.perf_counter()
    
    print(f"A* Execution time for {grid_size}x{grid_size} grid: {end_time - start_time:.4f} seconds")

if __name__ == "__main__":
    benchmark_astar()
