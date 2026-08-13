from path_planning.grid import Grid
from path_planning.a_star import astar
from path_planning.a_star_optimized import astar_optimized

def test_correctness():
    grid_size = 50
    g = Grid(grid_size, grid_size, obstacle_prob=0.1, seed=42)
    start = (0, 0)
    goal = (grid_size-1, grid_size-1)

    path1, _ = astar(g, start, goal)
    path2 = astar_optimized(g, start, goal)
    
    assert path1 == path2, "Paths do not match!"
    print("Correctness test passed!")

if __name__ == "__main__":
    test_correctness()
