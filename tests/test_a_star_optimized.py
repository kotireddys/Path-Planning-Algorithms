import pytest

numba = pytest.importorskip("numba", reason="a_star_optimized requires the optional 'perf' extra (numba)")

from path_planning.grid import Grid
from path_planning.a_star import astar
from path_planning.a_star_optimized import astar_optimized


def _cost(path):
    return len(path) - 1 if path else None


@pytest.mark.parametrize("width,height,obstacle_prob,seed,start,goal", [
    (10, 10, 0.0, 1, (0, 0), (9, 9)),
    (30, 30, 0.25, 2, (0, 0), (29, 29)),
    (10, 10, 0.0, 3, (2, 2), (2, 2)),  # start == goal
    (20, 20, 0.45, 4, (0, 0), (19, 19)),  # likely unreachable
    (80, 60, 0.2, 42, (2, 2), (77, 55)),  # matches examples/run_demo.py's astar demo
])
def test_matches_reference_astar(width, height, obstacle_prob, seed, start, goal):
    grid = Grid(width, height, obstacle_prob=obstacle_prob, seed=seed)
    grid.set_free(*start)
    grid.set_free(*goal)

    reference_path, _ = astar(grid, start, goal)
    optimized_path, _ = astar_optimized(grid, start, goal)

    assert (reference_path is None) == (optimized_path is None)
    # Compare cost, not the exact path: on a uniform-cost 4-connected grid
    # there are often multiple shortest paths, and the two implementations
    # break ties differently (different heap implementations).
    assert _cost(reference_path) == _cost(optimized_path)
