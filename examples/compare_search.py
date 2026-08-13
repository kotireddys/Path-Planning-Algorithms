"""Side-by-side comparison of A*, Dijkstra, Greedy Best-First, and D* Lite on
the same grid, stepping in lockstep so you can see how the priority function
(g+h vs g vs h) reshapes the search — and its cost.

Cells are shaded by g using a shared color scale across all four panels, so
the "wavefront" each algorithm grows is directly comparable: Dijkstra's is a
uniform ring, A*'s is stretched toward the goal, and Greedy's is a thin
tendril that mostly ignores cost. D* Lite searches backward from the goal,
so its g is cost-to-GOAL rather than cost-to-start like the other three —
its wavefront grows outward from the goal marker instead of the start one,
which is the point: that's what makes replanning near the start cheap.

Controls:
  -> / space   step all four forward one expansion each
  a            toggle autoplay
  r            reset
  q / esc      quit
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm

from path_planning.grid import Grid
from path_planning.a_star import search_steps
from path_planning.d_star_lite import d_star_lite_steps

MODES = ['astar', 'dijkstra', 'greedy', 'dstarlite']
TITLES = {
    'astar': 'A* (g + h)',
    'dijkstra': 'Dijkstra (g)',
    'greedy': 'Greedy Best-First (h)',
    'dstarlite': 'D* Lite (backward, cost-to-goal)',
}


class AlgoState:
    def __init__(self, grid, start, goal, mode):
        self.mode = mode
        if mode == 'dstarlite':
            self.gen = d_star_lite_steps(grid, start, goal)
        else:
            self.gen = search_steps(grid, start, goal, mode=mode)
        self.g_of = {}          # node -> g value, for every closed/frontier node seen
        self.frontier_nodes = []
        self.current = None
        self.path = None
        self.done = False
        self.success = None
        self.final = None

    def step(self):
        if self.done:
            return
        try:
            frame = next(self.gen)
        except StopIteration:
            self.done = True
            return
        if not frame['done']:
            self.current = frame['current']
            self.g_of[frame['current']] = frame['g']
            self.frontier_nodes = frame['frontier_nodes']
        else:
            self.done = True
            self.success = frame['success']
            self.path = frame.get('path')
            self.current = None
            self.frontier_nodes = []
            self.final = frame


class CompareViewer:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.base = grid.cells
        self.algos = {m: AlgoState(grid, start, goal, m) for m in MODES}
        self.autoplay = False

        self.fig, axes_grid = plt.subplots(2, 2, figsize=(11, 10))
        self.axes = axes_grid.flatten()
        self.ims = {}
        self.subtitles = {}
        h, w = self.base.shape
        obstacle_rgba = np.zeros((h, w, 4))
        obstacle_rgba[self.base == 1] = (0, 0, 0, 1)

        for ax, mode in zip(self.axes, MODES):
            ax.imshow(np.ones((h, w, 3)))  # white background
            ax.imshow(obstacle_rgba)  # obstacles on top
            im = ax.imshow(np.full((h, w, 4), 0.0), vmin=0, vmax=1)
            self.ims[mode] = im
            ax.plot(start[0], start[1], 's', color='green', markersize=6)
            ax.plot(goal[0], goal[1], 's', color='magenta', markersize=6)
            ax.set_xlim(-0.5, w - 0.5)
            ax.set_ylim(h - 0.5, -0.5)
            ax.set_xticks([])
            ax.set_yticks([])
            self.subtitles[mode] = ax.set_title('')

        self.cmap = matplotlib.colormaps['viridis']
        sm = cm.ScalarMappable(cmap=self.cmap)
        sm.set_array([])
        self.fig.subplots_adjust(left=0.05, right=0.95, top=0.88, bottom=0.12, wspace=0.1, hspace=0.25)
        cax = self.fig.add_axes((0.3, 0.04, 0.4, 0.02))
        self.fig.colorbar(sm, cax=cax, orientation='horizontal', label='g (cost-to-reach, or cost-to-goal for D* Lite)')

        self.suptitle = self.fig.suptitle('', fontsize=11, y=0.97)
        self.timer = self.fig.canvas.new_timer(interval=25)
        self.timer.add_callback(self._autoplay_tick)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.render()

    def max_g_seen(self):
        best = 1
        for a in self.algos.values():
            if a.g_of:
                best = max(best, max(a.g_of.values()))
        return best

    def render(self):
        vmax = self.max_g_seen()
        h, w = self.base.shape
        for mode in MODES:
            a = self.algos[mode]
            layer = np.zeros((h, w, 4))
            for (x, y), g in a.g_of.items():
                layer[y, x] = self.cmap(g / vmax)
            for (x, y) in a.frontier_nodes:
                if (x, y) not in a.g_of:
                    layer[y, x] = (1, 0.85, 0.2, 0.9)  # frontier not yet closed
            if a.current is not None:
                layer[a.current[1], a.current[0]] = (1, 0, 0, 1)
            self.ims[mode].set_data(layer)

            ax = self.axes[MODES.index(mode)]
            # clear any previous path line (keep start/goal markers, which are the
            # first two Line2D artists added in __init__)
            while len(ax.lines) > 2:
                ax.lines[-1].remove()
            if a.path:
                xs = [p[0] for p in a.path]
                ys = [p[1] for p in a.path]
                ax.plot(xs, ys, '-', color='white', linewidth=2.5)
                ax.plot(xs, ys, '-', color='deepskyblue', linewidth=1.2)

            if a.done and a.final is not None:
                if a.success:
                    self.subtitles[mode].set_text(
                        f"{TITLES[mode]}\ncost={a.final['path_cost']}  "
                        f"expanded={a.final['expanded']}  peak frontier={a.final['max_frontier']}"
                    )
                else:
                    self.subtitles[mode].set_text(f"{TITLES[mode]}\nno path found")
            else:
                expanded = len(a.g_of)
                self.subtitles[mode].set_text(
                    f"{TITLES[mode]}\nexpanded={expanded}  frontier={len(a.frontier_nodes)}"
                )

        state = 'autoplay' if self.autoplay else 'paused'
        self.suptitle.set_text(
            f"A* vs Dijkstra vs Greedy Best-First vs D* Lite ({state}) "
            "— ->/space step, a autoplay, r reset, q quit"
        )
        self.fig.canvas.draw_idle()

    def step_all(self):
        any_active = False
        for a in self.algos.values():
            if not a.done:
                a.step()
                any_active = True
        self.render()
        return any_active

    def reset(self):
        self.algos = {m: AlgoState(self.grid, self.start, self.goal, m) for m in MODES}
        self.autoplay = False
        self.timer.stop()
        self.render()

    def _autoplay_tick(self):
        if not self.step_all():
            self.autoplay = False
            self.timer.stop()

    def toggle_autoplay(self):
        self.autoplay = not self.autoplay
        if self.autoplay:
            self.timer.start()
        else:
            self.timer.stop()
        self.render()

    def on_key(self, event):
        if event.key in ('right', ' '):
            self.step_all()
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
    parser.add_argument('--obstacle-prob', type=float, default=0.25)
    parser.add_argument('--seed', type=int, default=24)
    parser.add_argument('--start', type=parse_xy, default=(2, 2))
    parser.add_argument('--goal', type=parse_xy, default=(77, 55))
    args = parser.parse_args()

    grid = Grid(args.width, args.height, obstacle_prob=args.obstacle_prob, seed=args.seed)
    grid.set_free(*args.start)
    grid.set_free(*args.goal)

    CompareViewer(grid, args.start, args.goal)
    plt.show()


if __name__ == '__main__':
    main()
