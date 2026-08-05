from path_planning.grid import Grid
from path_planning.rrt import rrt


def test_rrt_finds_path_on_empty_grid():
    g = Grid(20, 20, obstacle_prob=0.0, seed=3)
    start = (0, 0)
    goal = (19, 19)
    path, nodes, parent, visited = rrt(g, start, goal, max_iters=2000, step_size=5.0)
    assert path is not None
    assert len(path) >= 2
