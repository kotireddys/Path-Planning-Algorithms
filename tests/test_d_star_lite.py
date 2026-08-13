from path_planning.grid import Grid
from path_planning.d_star_lite import DStarLite, d_star_lite


def test_d_star_lite_shortest_path():
    g = Grid(10, 10, obstacle_prob=0.0, seed=2)
    start = (0, 0)
    goal = (9, 9)
    path, visited = d_star_lite(g, start, goal)
    assert path is not None
    # Manhattan distance is 18, path nodes should be distance+1 (matches astar())
    assert len(path) == 19


def test_d_star_lite_replans_after_new_obstacle():
    g = Grid(10, 10, obstacle_prob=0.0, seed=2)
    start, goal = (0, 0), (9, 9)
    planner = DStarLite(g, start, goal)
    planner.compute_shortest_path()
    path1 = planner.extract_path()
    assert path1 is not None

    blocked = path1[len(path1) // 2]
    assert blocked not in (start, goal)
    planner.set_obstacle(blocked)
    planner.compute_shortest_path()
    path2 = planner.extract_path()

    assert path2 is not None
    assert blocked not in path2
