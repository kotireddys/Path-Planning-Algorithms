"""Compares path_planning.a_star.astar() against the numba-JIT
path_planning.a_star_optimized.astar_optimized() across grid sizes.

Requires the optional 'perf' extra: pip install -e ".[perf]" (or `pip
install numba`). The very first call into a @njit function triggers a
compilation pass that can take much longer than the search itself, so that
warm-up call happens once per grid size, untimed, before any timing starts.
"""
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from path_planning.grid import Grid
from path_planning.a_star import astar

try:
    from path_planning.a_star_optimized import astar_optimized
except ImportError:
    print("numba is not installed — run `pip install -e '.[perf]'` first.")
    sys.exit(1)

GRID_SIZES = (100, 200, 400, 800)
SEED = 42
TRIALS = 7


def best_of(fn, trials=TRIALS):
    return min(_time_once(fn) for _ in range(trials))


def _time_once(fn):
    t0 = time.perf_counter()
    fn()
    return time.perf_counter() - t0


def main():
    print(f"{'grid':>10} {'original (ms)':>15} {'optimized (ms)':>16} {'speedup':>10}")
    for size in GRID_SIZES:
        grid = Grid(size, size, obstacle_prob=0.2, seed=SEED)
        start, goal = (0, 0), (size - 1, size - 1)
        grid.set_free(*start)
        grid.set_free(*goal)

        path, _ = astar(grid, start, goal)
        if path is None:
            print(f"{size}x{size}: no path at this seed, skipping")
            continue

        astar_optimized(grid, start, goal)  # warm up the JIT, untimed

        t_orig = best_of(lambda: astar(grid, start, goal))
        t_opt = best_of(lambda: astar_optimized(grid, start, goal))
        print(f"{size:>4}x{size:<4} {t_orig * 1000:>14.2f} {t_opt * 1000:>15.2f} "
              f"{t_orig / t_opt:>9.1f}x")


if __name__ == "__main__":
    main()
