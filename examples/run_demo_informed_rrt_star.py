import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from path_planning.grid import Grid
from path_planning.informed_rrt_star import informed_rrt_star
from visualize import frames_from_rrt, save_gif


def demo_informed_rrt_star(out_dir):
    grid = Grid(80, 60, obstacle_prob=0.2, seed=11)
    start = (5, 5)
    goal = (70, 50)
    grid.set_free(*start)
    grid.set_free(*goal)
    path, nodes, parent, visited = informed_rrt_star(grid, start, goal, max_iters=1500, step_size=3.0)
    frames = frames_from_rrt(grid, nodes, parent, visited, path_pts=path, steps_per_frame=10)
    out = os.path.join(out_dir, 'informed_rrt_star_demo.gif')
    save_gif(frames, out, fps=15)
    print('Wrote', out)
    if path:
        cost = sum(
            ((path[i][0] - path[i - 1][0]) ** 2 + (path[i][1] - path[i - 1][1]) ** 2) ** 0.5
            for i in range(1, len(path))
        )
        print(f'Path cost: {cost:.2f}, nodes explored: {len(nodes)}')
    else:
        print('No path found.')


if __name__ == '__main__':
    demo_informed_rrt_star('.')
