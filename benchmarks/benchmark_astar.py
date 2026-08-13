import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from path_planning.grid import Grid
from path_planning.a_star import astar


def benchmark_astar(grid_size=500, seed=42, trials=5):
    g = Grid(grid_size, grid_size, obstacle_prob=0.2, seed=seed)
    start = (0, 0)
    goal = (grid_size - 1, grid_size - 1)

    best = min(
        _time_once(lambda: astar(g, start, goal))
        for _ in range(trials)
    )
    print(f"A* on a {grid_size}x{grid_size} grid: {best * 1000:.2f} ms (best of {trials})")


def _time_once(fn):
    t0 = time.perf_counter()
    fn()
    return time.perf_counter() - t0


if __name__ == "__main__":
    benchmark_astar()
