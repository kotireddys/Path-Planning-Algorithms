import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from path_planning.grid import Grid
from path_planning.d_star_lite import DStarLite
from visualize import frames_from_astar, save_gif


def main():
    grid = Grid(80, 60, obstacle_prob=0.2, seed=42)
    start = (2, 2)
    goal = (77, 55)
    grid.set_free(*start)
    grid.set_free(*goal)

    planner = DStarLite(grid, start, goal)
    visited1 = planner.compute_shortest_path()
    path1 = planner.extract_path()
    print(f'Initial path: {len(path1)} steps, cost {planner.g[start]:.0f}, '
          f'{len(visited1)} vertices expanded')
    frames1 = frames_from_astar(grid, visited1, path1)
    save_gif(frames1, os.path.join('.', 'd_star_lite_initial.gif'), fps=15)
    print('Wrote d_star_lite_initial.gif')

    # Block a cell partway along the found path and replan incrementally.
    # This is the whole point of D* Lite: compute_shortest_path() only
    # reprocesses vertices whose cost actually changed, instead of
    # rerunning the search from scratch like calling astar() again would.
    blocked = path1[len(path1) // 2]
    planner.set_obstacle(blocked)
    visited2 = planner.compute_shortest_path()
    path2 = planner.extract_path()
    print(f'Blocked {blocked}; replanned path: {len(path2)} steps, cost {planner.g[start]:.0f}, '
          f'{len(visited2)} vertices reprocessed (vs {len(visited1)} for the original full search)')
    frames2 = frames_from_astar(grid, visited2, path2)
    save_gif(frames2, os.path.join('.', 'd_star_lite_replanned.gif'), fps=15)
    print('Wrote d_star_lite_replanned.gif')


if __name__ == '__main__':
    main()
