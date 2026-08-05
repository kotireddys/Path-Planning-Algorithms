from path_planning.grid import Grid


def test_grid_all_free():
    g = Grid(10, 10, obstacle_prob=0.0, seed=1)
    for x in range(10):
        for y in range(10):
            assert g.is_free(x, y)
