import os

import matplotlib

if not os.environ.get('MPLBACKEND'):
    # Frame generation here never calls plt.show(); the interactive backends
    # (e.g. TkAgg) spin up a real window toolkit per-frame and are drastically
    # slower (minutes vs. seconds for a full animation) than the headless Agg
    # backend, which is all this module needs.
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
import imageio
from matplotlib import colors


def frames_from_astar(grid, visited_order, path, figsize=(6, 6)):
    frames = []
    cmap = colors.ListedColormap(['white', 'black', 'green', 'red', 'cyan'])
    # 0 free,1 obstacle,2 start,3 goal,4 visited/path overlay
    for i, _ in enumerate(visited_order):
        arr = grid.cells.copy()
        for (x, y) in visited_order[: i + 1]:
            arr[y, x] = 4
        if path:
            for (x, y) in path:
                arr[y, x] = 4
        fig = plt.figure(figsize=figsize)
        plt.imshow(arr, cmap=cmap)
        plt.axis('off')
        fig.canvas.draw()
        buf = fig.canvas.buffer_rgba()
        w, h = fig.canvas.get_width_height()
        image = np.frombuffer(buf, dtype=np.uint8).reshape((h, w, 4))[..., :3]
        frames.append(image)
        plt.close(fig)
    return frames


def frames_from_rrt(grid, nodes, parent, visited_order, path_pts=None, figsize=(6, 6)):
    frames = []
    for k in range(1, len(visited_order) + 1):
        fig, ax = plt.subplots(figsize=figsize)
        ax.imshow(grid.cells, cmap='gray_r')
        # draw tree up to k
        used = set()
        for i, n in enumerate(nodes):
            if i == 0:
                continue
            if i - 1 >= len(visited_order):
                break
            p = parent.get(i)
            if p is None:
                continue
            x1, y1 = nodes[p]
            x2, y2 = nodes[i]
            ax.plot([x1, x2], [y1, y2], color='cyan', linewidth=0.8)
        if path_pts:
            xs = [p[0] for p in path_pts]
            ys = [p[1] for p in path_pts]
            ax.plot(xs, ys, color='red', linewidth=2)
        ax.set_axis_off()
        fig.canvas.draw()
        buf = fig.canvas.buffer_rgba()
        w, h = fig.canvas.get_width_height()
        image = np.frombuffer(buf, dtype=np.uint8).reshape((h, w, 4))[..., :3]
        frames.append(image)
        plt.close(fig)
    return frames


def frames_from_dubins(grid, path_pts, figsize=(6, 6), steps_per_frame: int = 4):
    frames = []
    pts = path_pts
    if not pts:
        return frames
    # sample along the path and create frames showing progression
    total = len(pts)
    idx = 0
    while idx < total:
        fig, ax = plt.subplots(figsize=figsize)
        ax.imshow(grid.cells, cmap='gray_r')
        xs = [p[0] for p in pts[: idx + 1]]
        ys = [p[1] for p in pts[: idx + 1]]
        ax.plot(xs, ys, color='red', linewidth=2)
        ax.set_axis_off()
        fig.canvas.draw()
        buf = fig.canvas.buffer_rgba()
        w, h = fig.canvas.get_width_height()
        image = np.frombuffer(buf, dtype=np.uint8).reshape((h, w, 4))[..., :3]
        frames.append(image)
        plt.close(fig)
        idx += steps_per_frame
    return frames


def save_gif(frames, path: str, fps: int = 10):
    imageio.mimsave(path, frames, fps=fps)
