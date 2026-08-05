from path_planning.grid3d import Grid3D
from path_planning.a_star_3d import astar_3d

def test_astar_3d():
    grid = Grid3D(10, 10, 10, obstacle_prob=0.0, seed=42)
    start = (0, 0, 0)
    goal = (5, 5, 5)
    path, visited = astar_3d(grid, start, goal)
    
    assert path is not None
    assert path[0] == start
    assert path[-1] == goal
    print("3D A* Test Passed!")

if __name__ == "__main__":
    test_astar_3d()
