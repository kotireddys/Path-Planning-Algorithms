import argparse
import os
import random
import sys
from pathlib import Path

# ensure repo root is on sys.path when running this script from the examples/ folder
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from path_planning.grid import Grid
from path_planning.a_star import astar
from path_planning.rrt import rrt
from visualize import frames_from_astar, frames_from_rrt, frames_from_dubins, save_gif


def ensure_free(grid, pt):
    x, y = pt
    grid.set_free(x, y)


def demo_astar(out_dir):
    grid = Grid(80, 60, obstacle_prob=0.2, seed=42)
    start = (2, 2)
    goal = (77, 55)
    ensure_free(grid, start)
    ensure_free(grid, goal)
    path, visited = astar(grid, start, goal)
    frames = frames_from_astar(grid, visited, path)
    out = os.path.join(out_dir, 'astar_demo.gif')
    save_gif(frames, out, fps=15)
    print('Wrote', out)


def demo_rrt(out_dir):
    grid = Grid(80, 60, obstacle_prob=0.2, seed=7)
    start = (5, 5)
    goal = (70, 50)
    ensure_free(grid, start)
    ensure_free(grid, goal)
    path, nodes, parent, visited = rrt(grid, start, goal, max_iters=1200, step_size=3.0)
    frames = frames_from_rrt(grid, nodes, parent, visited, path_pts=path)
    out = os.path.join(out_dir, 'rrt_demo.gif')
    save_gif(frames, out, fps=15)
    print('Wrote', out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--algo', choices=['astar', 'rrt', 'rrtstar', 'dubins'], default='astar')
    parser.add_argument('--out', default='.')
    args = parser.parse_args()
    Path(args.out).mkdir(parents=True, exist_ok=True)
    if args.algo == 'astar':
        demo_astar(args.out)
    elif args.algo == 'rrt':
        demo_rrt(args.out)
    elif args.algo == 'rrtstar':
        demo_rrtstar(args.out)
    elif args.algo == 'dubins':
        demo_dubins(args.out)


def demo_rrtstar(out_dir):
    from path_planning.rrt_star import rrt_star
    grid = Grid(80, 60, obstacle_prob=0.2, seed=11)
    start = (5, 5)
    goal = (70, 50)
    ensure_free(grid, start)
    ensure_free(grid, goal)
    path, nodes, parent, visited = rrt_star(grid, start, goal, max_iters=1500, step_size=3.0)
    frames = frames_from_rrt(grid, nodes, parent, visited, path_pts=path)
    out = os.path.join(out_dir, 'rrtstar_demo.gif')
    save_gif(frames, out, fps=15)
    print('Wrote', out)


def demo_dubins(out_dir):
    from path_planning.dubins import dubins_path
    grid = Grid(80, 60, obstacle_prob=0.15, seed=5)
    start_pose = (10.0, 10.0, 0.0)
    goal_pose = (70.0, 45.0, 0.0)
    path_pts = dubins_path(start_pose, goal_pose, turning_radius=6.0, step_size=0.8)
    frames = frames_from_dubins(grid, path_pts)
    out = os.path.join(out_dir, 'dubins_demo.gif')
    save_gif(frames, out, fps=15)
    print('Wrote', out)


if __name__ == '__main__':
    main()
