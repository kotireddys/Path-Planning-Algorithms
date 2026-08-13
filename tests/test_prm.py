from path_planning.grid import Grid
from path_planning.prm import prm


def test_prm_finds_path_on_empty_grid():
    g = Grid(20, 20, obstacle_prob=0.0, seed=3)
    start = (0, 0)
    goal = (19, 19)
    path, nodes, edges, visited = prm(g, start, goal, num_samples=150, connect_radius=8.0, seed=1)
    assert path is not None
    assert len(path) >= 2
