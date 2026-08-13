"""Interactive, step-through viewer for A*.

Steps through the exact same search as path_planning.a_star.astar(), one
expansion at a time, showing the open set (frontier), closed set (expanded
nodes), and the live cost values (g, h, f) driving each decision.

Controls:
  -> / space   step forward one expansion
  <-           step backward (re-inspect a previous expansion)
  a            toggle autoplay
  r            reset to the start
  q / esc      quit
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib import colors

from path_planning.grid import Grid
from path_planning.a_star import astar_steps

CMAP = colors.ListedColormap(
    ['white', 'black', 'green', 'magenta', 'lightblue', 'gold', 'red', 'cyan']
)
# 0 free, 1 obstacle, 2 start, 3 goal, 4 closed, 5 frontier, 6 current, 7 path


def build_display(base, start, goal, closed, frontier, current, path=None):
    arr = base.copy()
    for (x, y) in closed:
        if arr[y, x] != 1:
            arr[y, x] = 4
    for (x, y) in frontier:
        if arr[y, x] != 1:
            arr[y, x] = 5
    if path:
        for (x, y) in path:
            if arr[y, x] != 1:
                arr[y, x] = 7
    if current is not None and arr[current[1], current[0]] != 1:
        arr[current[1], current[0]] = 6
    arr[start[1], start[0]] = 2
    arr[goal[1], goal[0]] = 3
    return arr


def metrics_text(frame):
    if frame is None:
        return "Press -> or space to start stepping through A*."
    if not frame['done']:
        return (
            f"Iteration: {frame['iteration']}\n"
            f"Current node: {frame['current']}\n"
            f"g (cost so far): {frame['g']}\n"
            f"h (heuristic to goal): {frame['h']}\n"
            f"f = g + h: {frame['f']}\n"
            f"Frontier size: {len(frame['frontier_nodes'])}\n"
            f"Nodes expanded: {frame['expanded']}\n"
            f"Max frontier seen: {frame['max_frontier']}"
        )
    if frame['success']:
        return (
            f"DONE - path found\n"
            f"Path length: {len(frame['path'])} steps\n"
            f"Path cost: {frame['path_cost']}\n"
            f"Nodes expanded: {frame['expanded']}\n"
            f"Max frontier size: {frame['max_frontier']}\n"
            f"Time: {frame['elapsed'] * 1000:.2f} ms"
        )
    return (
        f"DONE - no path found\n"
        f"Nodes expanded: {frame['expanded']}\n"
        f"Max frontier size: {frame['max_frontier']}\n"
        f"Time: {frame['elapsed'] * 1000:.2f} ms"
    )


class Viewer:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.base = grid.cells.copy()
        self.closed_history = []  # nodes in the order they were expanded
        self.frames = []          # every yielded dict, in order
        self.index = -1           # position into self.frames
        self.gen = astar_steps(grid, start, goal)
        self.exhausted = False
        self.autoplay = False

        self.fig, (self.ax_grid, self.ax_text) = plt.subplots(
            1, 2, figsize=(11, 6), gridspec_kw={'width_ratios': [3, 1.4]}
        )
        self.ax_text.axis('off')
        self.text_artist = self.ax_text.text(
            0.02, 0.98, '', va='top', ha='left', family='monospace', fontsize=10,
            transform=self.ax_text.transAxes,
        )
        self.im = self.ax_grid.imshow(self.base, cmap=CMAP, vmin=0, vmax=7)
        self.ax_grid.axis('off')

        self.timer = self.fig.canvas.new_timer(interval=40)
        self.timer.add_callback(self._autoplay_tick)

        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.render()

    def current_frame(self):
        return self.frames[self.index] if self.index >= 0 else None

    def closed_and_frontier_for(self, i):
        if i < 0:
            return set(), set()
        frame = self.frames[i]
        closed = set(self.closed_history[:frame['closed_len']])
        frontier = set(frame['frontier_nodes']) if not frame['done'] else set()
        return closed, frontier

    def render(self):
        frame = self.current_frame()
        closed, frontier = self.closed_and_frontier_for(self.index)
        current = frame['current'] if (frame and not frame['done']) else None
        path = frame['path'] if (frame and frame['done'] and frame['success']) else None
        arr = build_display(self.base, self.start, self.goal, closed, frontier, current, path)
        self.im.set_data(arr)
        self.text_artist.set_text(metrics_text(frame))
        state = "autoplay" if self.autoplay else "paused"
        self.ax_grid.set_title(
            f"A* step-through ({state}) — ->/space step, a autoplay, r reset, q quit"
        )
        self.fig.canvas.draw_idle()

    def step_forward(self):
        if self.index + 1 < len(self.frames):
            self.index += 1
            self.render()
            return True
        if self.exhausted:
            return False
        try:
            frame = next(self.gen)
        except StopIteration:
            self.exhausted = True
            return False
        if not frame['done']:
            self.closed_history.append(frame['current'])
        frame['closed_len'] = len(self.closed_history)
        self.frames.append(frame)
        self.index += 1
        if frame['done']:
            self.exhausted = True
        self.render()
        return True

    def step_backward(self):
        if self.index > 0:
            self.index -= 1
            self.render()

    def reset(self):
        self.gen = astar_steps(self.grid, self.start, self.goal)
        self.closed_history = []
        self.frames = []
        self.index = -1
        self.exhausted = False
        self.autoplay = False
        self.timer.stop()
        self.render()

    def _autoplay_tick(self):
        if not self.step_forward():
            self.autoplay = False
            self.timer.stop()
            self.render()

    def toggle_autoplay(self):
        self.autoplay = not self.autoplay
        if self.autoplay:
            self.timer.start()
        else:
            self.timer.stop()
        self.render()

    def on_key(self, event):
        if event.key in ('right', ' '):
            self.step_forward()
        elif event.key == 'left':
            self.step_backward()
        elif event.key == 'a':
            self.toggle_autoplay()
        elif event.key == 'r':
            self.reset()
        elif event.key in ('q', 'escape'):
            plt.close(self.fig)


def parse_xy(s):
    x, y = s.split(',')
    return (int(x), int(y))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--width', type=int, default=80)
    parser.add_argument('--height', type=int, default=60)
    parser.add_argument('--obstacle-prob', type=float, default=0.2)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--start', type=parse_xy, default=(2, 2))
    parser.add_argument('--goal', type=parse_xy, default=(77, 55))
    args = parser.parse_args()

    grid = Grid(args.width, args.height, obstacle_prob=args.obstacle_prob, seed=args.seed)
    grid.set_free(*args.start)
    grid.set_free(*args.goal)

    Viewer(grid, args.start, args.goal)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
