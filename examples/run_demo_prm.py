import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from path_planning.grid import Grid
from path_planning.prm import prm
from visualize import frames_from_prm, save_gif


def demo_prm(out_dir):
    grid = Grid(80, 60, obstacle_prob=0.2, seed=13)
    start = (5, 5)
    goal = (70, 50)
    grid.set_free(*start)
    grid.set_free(*goal)
    path, nodes, edges, visited = prm(grid, start, goal, num_samples=300, connect_radius=10.0, seed=13)
    frames = frames_from_prm(grid, nodes, edges, path_pts=path)
    out = os.path.join(out_dir, 'prm_demo.gif')
    save_gif(frames, out, fps=10)
    print('Wrote', out)
    if path is None:
        print('No path found.')
    else:
        print(f'Roadmap nodes: {len(nodes)}, path length: {len(path)}')


if __name__ == '__main__':
    demo_prm('.')
