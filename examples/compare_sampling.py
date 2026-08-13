"""Side-by-side comparison of RRT*, Informed RRT*, and PRM on the same grid.

Unlike compare_search.py's algorithms, these are sampling-based and have no
per-expansion step generator — each planner runs once to completion here,
and the panels then reveal its result incrementally so growth is still
visible: RRT* and Informed RRT* reveal their tree edge by edge (in the order
nodes were added), which is what makes the payoff of informed sampling
visible — after Informed RRT* finds a first solution it keeps growing, but
only inside the shrinking ellipse, so its tree stays sparse outside a narrow
corridor around the best path while RRT*'s keeps spreading everywhere. PRM's
roadmap is built upfront (not grown query by query), so its panel shows the
full roadmap immediately and instead reveals the Dijkstra path across it
point by point.

Controls:
  -> / space   step all three forward one reveal each
  a            toggle autoplay
  r            reset (re-samples each planner from scratch)
  q / esc      quit
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

from path_planning.grid import Grid
from path_planning.rrt import distance
from path_planning.rrt_star import rrt_star
from path_planning.informed_rrt_star import informed_rrt_star
from path_planning.prm import prm

MODES = ['rrt_star', 'informed_rrt_star', 'prm']
TITLES = {
    'rrt_star': 'RRT*',
    'informed_rrt_star': 'Informed RRT* (ellipsoidal sampling)',
    'prm': 'PRM (roadmap built upfront, path revealed)',
}


def path_cost(path):
    if not path:
        return None
    return sum(distance(path[i - 1], path[i]) for i in range(1, len(path)))


class SamplingAlgoState:
    """Runs its planner once to completion, then reveals the result
    incrementally: nodes/edges in growth order for the tree-based planners,
    path points for PRM (whose roadmap doesn't grow query by query)."""

    def __init__(self, grid, start, goal, mode):
        self.mode = mode
        self.edges = None
        self.parent = None
        if mode == 'rrt_star':
            self.path, self.nodes, self.parent, _ = rrt_star(
                grid, start, goal, max_iters=1500, step_size=3.0)
        elif mode == 'informed_rrt_star':
            self.path, self.nodes, self.parent, _ = informed_rrt_star(
                grid, start, goal, max_iters=1500, step_size=3.0)
        else:
            self.path, self.nodes, self.edges, _ = prm(
                grid, start, goal, num_samples=300, connect_radius=10.0, seed=13)

        if mode == 'prm':
            self.total_steps = len(self.path) if self.path else 1
        else:
            self.total_steps = max(1, len(self.nodes) - 1)
        self.reveal_per_step = max(1, self.total_steps // 150)

        self.revealed = 0
        self.done = False
        self.segments = []       # tree edges revealed so far, as [(pt, pt), ...]
        self.lines_drawn = 0     # how many tree nodes' worth of segments is already in self.segments
        self.path_drawn_to = 0   # how many path points are already reflected in the path line

    def step(self):
        if self.done:
            return
        self.revealed = min(self.total_steps, self.revealed + self.reveal_per_step)
        if self.revealed >= self.total_steps:
            self.done = True


class SamplingCompareViewer:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.autoplay = False
        self.algos = {m: SamplingAlgoState(grid, start, goal, m) for m in MODES}

        self.fig, self.axes = plt.subplots(1, 3, figsize=(16, 5.6))
        self.subtitles = {}
        self.edge_collections = {}
        for ax, mode in zip(self.axes, MODES):
            ax.imshow(grid.cells, cmap='gray_r')
            ax.plot(start[0], start[1], 's', color='green', markersize=6)
            ax.plot(goal[0], goal[1], 's', color='magenta', markersize=6)
            ax.set_xticks([])
            ax.set_yticks([])
            self.subtitles[mode] = ax.set_title('')
            # Edges are drawn as one LineCollection per panel and updated via
            # set_segments() rather than as individual ax.plot() Line2D
            # artists — PRM's roadmap alone can have thousands of edges, and
            # replotting/removing that many artists every render is what
            # made the (matplotlib-artist-count)^2 removal cost blow up.
            lw, alpha = (0.3, 0.4) if mode == 'prm' else (0.8, 1.0)
            coll = LineCollection([], colors='cyan', linewidths=lw, alpha=alpha)
            ax.add_collection(coll)
            self.edge_collections[mode] = coll

        self.fig.subplots_adjust(left=0.02, right=0.98, top=0.8, bottom=0.05, wspace=0.05)
        self.suptitle = self.fig.suptitle('', fontsize=11, y=0.96)
        self.timer = self.fig.canvas.new_timer(interval=25)
        self.timer.add_callback(self._autoplay_tick)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self._draw_static_roadmap()
        self.render()

    def _draw_static_roadmap(self):
        # PRM's roadmap is built upfront and never grows, so it only needs
        # to be drawn once, rather than every render() call like the
        # incrementally-growing RRT*/Informed RRT* trees.
        a = self.algos['prm']
        segments = []
        for i, neighbors in a.edges.items():
            for j, _ in neighbors:
                if j > i:
                    segments.append((a.nodes[i], a.nodes[j]))
        self.edge_collections['prm'].set_segments(segments)

    def _clear_path_lines(self, ax):
        # keep start/goal markers, the first two Line2D artists added in __init__
        while len(ax.lines) > 2:
            ax.lines[-1].remove()

    def _draw_path(self, ax, xs, ys):
        self._clear_path_lines(ax)
        ax.plot(xs, ys, '-', color='white', linewidth=2.5)
        ax.plot(xs, ys, '-', color='red', linewidth=1.4)

    def _render_tree(self, ax, a):
        if a.revealed > a.lines_drawn:
            for i in range(a.lines_drawn + 1, a.revealed + 1):
                p = a.parent.get(i)
                if p is not None:
                    a.segments.append((a.nodes[p], a.nodes[i]))
            a.lines_drawn = a.revealed
            self.edge_collections[a.mode].set_segments(a.segments)
        if a.done and a.path and a.path_drawn_to == 0:
            xs = [p[0] for p in a.path]
            ys = [p[1] for p in a.path]
            self._draw_path(ax, xs, ys)
            a.path_drawn_to = len(a.path)

    def _render_prm(self, ax, a):
        if a.path and a.revealed != a.path_drawn_to:
            xs = [p[0] for p in a.path[:a.revealed]]
            ys = [p[1] for p in a.path[:a.revealed]]
            self._draw_path(ax, xs, ys)
            a.path_drawn_to = a.revealed

    def render(self):
        for ax, mode in zip(self.axes, MODES):
            a = self.algos[mode]
            if mode == 'prm':
                self._render_prm(ax, a)
            else:
                self._render_tree(ax, a)

            if mode == 'prm':
                edge_count = sum(len(v) for v in a.edges.values()) // 2
                base = f"roadmap: {len(a.nodes)} nodes, {edge_count} edges"
                if a.path is None:
                    self.subtitles[mode].set_text(f"{TITLES[mode]}\n{base}\nno path found")
                elif a.done:
                    self.subtitles[mode].set_text(
                        f"{TITLES[mode]}\n{base}\npath cost={path_cost(a.path):.1f}  points={len(a.path)}")
                else:
                    self.subtitles[mode].set_text(
                        f"{TITLES[mode]}\n{base}\nrevealing path: {a.revealed}/{a.total_steps}")
            else:
                if not a.done:
                    self.subtitles[mode].set_text(
                        f"{TITLES[mode]}\ngrowing tree: {a.revealed}/{a.total_steps} nodes")
                elif a.path is None:
                    self.subtitles[mode].set_text(f"{TITLES[mode]}\nno path found ({len(a.nodes)} nodes)")
                else:
                    self.subtitles[mode].set_text(
                        f"{TITLES[mode]}\npath cost={path_cost(a.path):.1f}  nodes={len(a.nodes)}")

        state = 'autoplay' if self.autoplay else 'paused'
        self.suptitle.set_text(
            f"RRT* vs Informed RRT* vs PRM ({state}) "
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
        self.algos = {m: SamplingAlgoState(self.grid, self.start, self.goal, m) for m in MODES}
        for mode in ('rrt_star', 'informed_rrt_star'):
            self.edge_collections[mode].set_segments([])
        for ax in self.axes:
            self._clear_path_lines(ax)
        self._draw_static_roadmap()
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
    parser.add_argument('--obstacle-prob', type=float, default=0.2)
    parser.add_argument('--seed', type=int, default=11)
    parser.add_argument('--start', type=parse_xy, default=(5, 5))
    parser.add_argument('--goal', type=parse_xy, default=(70, 50))
    args = parser.parse_args()

    grid = Grid(args.width, args.height, obstacle_prob=args.obstacle_prob, seed=args.seed)
    grid.set_free(*args.start)
    grid.set_free(*args.goal)

    SamplingCompareViewer(grid, args.start, args.goal)
    plt.show()


if __name__ == '__main__':
    main()
