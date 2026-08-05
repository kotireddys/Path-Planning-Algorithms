from path_planning.grid import Grid
from path_planning.a_star import astar


def test_astar_shortest_path():
    g = Grid(10, 10, obstacle_prob=0.0, seed=2)
    start = (0, 0)
    goal = (9, 9)
    path, visited = astar(g, start, goal)
    assert path is not None
    # Manhattan distance is 18, path nodes should be distance+1
    assert len(path) == 19
