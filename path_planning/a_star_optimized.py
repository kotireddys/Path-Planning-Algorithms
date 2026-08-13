"""JIT-compiled A*, functionally equivalent to path_planning.a_star.astar().

numba can't compile Python's heapq (it operates on arbitrary Python objects),
so getting an actual speedup means never leaving compiled code during the
search: the open set is a hand-rolled binary min-heap over parallel numpy
arrays, the grid is a flat uint8 array, and every node is a flat
`y * width + x` index instead of a tuple. All of that lives in `_astar_core`,
which is the only function that needs to be fast — it's the one decorated
with @njit. A thin, undecorated `astar_optimized()` wrapper handles
translating to/from the (x, y) tuples the rest of the codebase uses.

This module is optional: it requires `numba`, which is NOT a dependency of
the base package (see the `perf` extra in pyproject.toml) because it's a
~60MB LLVM-backed compiler toolchain that only pays for itself on large
grids. Import it lazily / behind a try-except if you use it from code that
must work without numba installed.
"""
import numpy as np
from numba import njit


@njit(cache=True)
def _heap_push(heap_f, heap_idx, size, f, idx):
    heap_f[size] = f
    heap_idx[size] = idx
    i = size
    while i > 0:
        parent = (i - 1) // 2
        if heap_f[parent] <= heap_f[i]:
            break
        heap_f[parent], heap_f[i] = heap_f[i], heap_f[parent]
        heap_idx[parent], heap_idx[i] = heap_idx[i], heap_idx[parent]
        i = parent
    return size + 1


@njit(cache=True)
def _heap_pop(heap_f, heap_idx, size):
    top_f = heap_f[0]
    top_idx = heap_idx[0]
    size -= 1
    heap_f[0] = heap_f[size]
    heap_idx[0] = heap_idx[size]
    i = 0
    while True:
        left = 2 * i + 1
        right = 2 * i + 2
        smallest = i
        if left < size and heap_f[left] < heap_f[smallest]:
            smallest = left
        if right < size and heap_f[right] < heap_f[smallest]:
            smallest = right
        if smallest == i:
            break
        heap_f[i], heap_f[smallest] = heap_f[smallest], heap_f[i]
        heap_idx[i], heap_idx[smallest] = heap_idx[smallest], heap_idx[i]
        i = smallest
    return top_f, top_idx, size


@njit(cache=True)
def _astar_core(obstacles, width, height, sx, sy, gx, gy):
    """4-connected grid A* entirely in compiled code. obstacles is a flat
    (height*width,) uint8 array, 1 = blocked. Returns (came_from, goal_idx);
    goal_idx is -1 if no path was found."""
    size = width * height
    start_idx = sy * width + sx
    goal_idx = gy * width + gx

    INF = np.float64(1e18)
    g_score = np.full(size, INF, dtype=np.float64)
    g_score[start_idx] = 0.0
    came_from = np.full(size, -1, dtype=np.int64)
    closed = np.zeros(size, dtype=np.uint8)

    # Each of the size cells can be relaxed (pushed) at most once per
    # incoming edge, and each cell has at most 4 neighbors, so this is a
    # safe upper bound on how many entries the heap will ever hold.
    cap = 4 * size + 1
    heap_f = np.empty(cap, dtype=np.float64)
    heap_idx = np.empty(cap, dtype=np.int64)
    heap_size = 0
    heap_size = _heap_push(heap_f, heap_idx, heap_size,
                            float(abs(sx - gx) + abs(sy - gy)), start_idx)

    dxs = np.array([1, -1, 0, 0], dtype=np.int64)
    dys = np.array([0, 0, 1, -1], dtype=np.int64)

    while heap_size > 0:
        _, idx, heap_size = _heap_pop(heap_f, heap_idx, heap_size)
        if closed[idx]:
            continue
        closed[idx] = 1
        if idx == goal_idx:
            return came_from, goal_idx

        cx = idx % width
        cy = idx // width
        for k in range(4):
            nx = cx + dxs[k]
            ny = cy + dys[k]
            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                continue
            nidx = ny * width + nx
            if obstacles[nidx]:
                continue
            tentative_g = g_score[idx] + 1.0
            if tentative_g < g_score[nidx]:
                g_score[nidx] = tentative_g
                came_from[nidx] = idx
                h = abs(nx - gx) + abs(ny - gy)
                heap_size = _heap_push(heap_f, heap_idx, heap_size,
                                        tentative_g + h, nidx)

    return came_from, -1


def astar_optimized(grid, start, goal):
    """Drop-in replacement for path_planning.a_star.astar() — same
    (path, visited) shape, path is None if unreachable. `visited` is only
    approximated (start/goal only) since the compiled core doesn't track
    per-node visit order; nothing in this codebase depends on its contents
    for anything other than truthiness/length in the demos that use plain
    astar(), and those don't call this function."""
    width, height = grid.width, grid.height
    obstacles = np.ascontiguousarray(grid.cells.reshape(-1)).astype(np.uint8)
    sx, sy = start
    gx, gy = goal

    came_from, goal_idx = _astar_core(obstacles, width, height, sx, sy, gx, gy)

    if goal_idx == -1:
        return None, [start]

    path = []
    idx = goal_idx
    while idx != -1:
        path.append((idx % width, idx // width))
        idx = came_from[idx]
    path.reverse()
    return path, [start, goal]
