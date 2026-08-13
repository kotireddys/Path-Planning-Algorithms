import time
from path_planning.grid import Grid
from path_planning.a_star import astar
from path_planning.a_star_optimized import astar_optimized

def benchmark():
    grid_size = 500
    g = Grid(grid_size, grid_size, obstacle_prob=0.2, seed=42)
    start = (0, 0)
    goal = (grid_size-1, grid_size-1)

    # Time original A*
    start_time = time.perf_counter()
    astar(g, start, goal)
    end_time = time.perf_counter()
    print(f"Original A* time: {end_time - start_time:.4f} seconds")
    
    # Time optimized A*
    start_time = time.perf_counter()
    astar_optimized(g, start, goal)
    end_time = time.perf_counter()
    print(f"Optimized A* time: {end_time - start_time:.4f} seconds")

if __name__ == "__main__":
    benchmark()
