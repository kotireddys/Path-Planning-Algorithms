import heapq
import time
from typing import Tuple, Dict, List, Optional

from .a_star import heuristic


class DStarLite:
    """Incremental replanning search (Koenig & Likhachev, 2002) on a 4-connected
    grid. Searches backward from goal to start, maintaining g (cost-to-goal)
    and rhs (one-step lookahead of g) for every vertex it has touched.

    The payoff over calling astar() again after the environment changes:
    set_obstacle() only re-queues the vertices whose cost could actually be
    affected, so compute_shortest_path() after a change reprocesses a small
    local neighborhood instead of the whole grid.
    """

    def __init__(self, grid, start: Tuple[int, int], goal: Tuple[int, int]):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.g: Dict[Tuple[int, int], float] = {}
        self.rhs: Dict[Tuple[int, int], float] = {}
        self.km = 0.0
        self.open_set: List[Tuple[Tuple[float, float], Tuple[int, int]]] = []
        self.open_entries: Dict[Tuple[int, int], Tuple[float, float]] = {}

        self.rhs[goal] = 0.0
        self._push(goal)

    def _calc_key(self, node):
        g = self.g.get(node, float('inf'))
        rhs = self.rhs.get(node, float('inf'))
        m = min(g, rhs)
        return (m + heuristic(self.start, node) + self.km, m)

    def _push(self, node):
        key = self._calc_key(node)
        heapq.heappush(self.open_set, (key, node))
        self.open_entries[node] = key

    def _neighbors(self, node):
        return self.grid.neighbors4(*node)

    def _neighbors_ignoring_blocked(self, node):
        x, y = node
        out = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if self.grid.in_bounds(nx, ny):
                out.append((nx, ny))
        return out

    def _cost(self, a, b):
        if not self.grid.is_free(*a) or not self.grid.is_free(*b):
            return float('inf')
        return 1.0

    def update_vertex(self, u):
        if u != self.goal:
            self.rhs[u] = min(
                (self._cost(u, s) + self.g.get(s, float('inf')) for s in self._neighbors(u)),
                default=float('inf'),
            )
        if self.g.get(u, float('inf')) != self.rhs.get(u, float('inf')):
            self._push(u)

    def compute_shortest_path(self):
        """Process the priority queue until start's g-value has converged.
        Returns the list of vertices expanded during this call."""
        expanded = []
        while self.open_set and (
            self.open_set[0][0] < self._calc_key(self.start)
            or self.rhs.get(self.start, float('inf')) != self.g.get(self.start, float('inf'))
        ):
            k_old, u = heapq.heappop(self.open_set)
            if self.open_entries.get(u) != k_old:
                continue  # stale heap entry, superseded by a later update_vertex
            del self.open_entries[u]
            expanded.append(u)
            k_new = self._calc_key(u)
            if k_old < k_new:
                self._push(u)
            elif self.g.get(u, float('inf')) > self.rhs.get(u, float('inf')):
                self.g[u] = self.rhs[u]
                for s in self._neighbors(u):
                    self.update_vertex(s)
            else:
                self.g[u] = float('inf')
                self.update_vertex(u)
                for s in self._neighbors(u):
                    self.update_vertex(s)
        return expanded

    def extract_path(self) -> Optional[List[Tuple[int, int]]]:
        """Greedily follow decreasing g toward the goal. g[node] is cost
        from node to goal (backward search), so this walks start -> goal."""
        if self.g.get(self.start, float('inf')) == float('inf'):
            return None
        path = [self.start]
        current = self.start
        seen = {self.start}
        while current != self.goal:
            neighbors = self._neighbors(current)
            if not neighbors:
                return None
            current = min(neighbors, key=lambda s: self._cost(current, s) + self.g.get(s, float('inf')))
            if self.g.get(current, float('inf')) == float('inf') or current in seen:
                return None
            seen.add(current)
            path.append(current)
        return path

    def set_obstacle(self, node: Tuple[int, int]):
        """Mark a cell as newly blocked and queue the vertices whose cost
        this could change. Call compute_shortest_path() afterward to
        actually propagate the update."""
        self.grid.cells[node[1], node[0]] = 1
        self.update_vertex(node)
        for s in self._neighbors_ignoring_blocked(node):
            self.update_vertex(s)


def d_star_lite(grid, start: Tuple[int, int], goal: Tuple[int, int]):
    """One-shot convenience wrapper matching astar()'s (path, visited_order)
    signature. For incremental replanning, use the DStarLite class directly."""
    planner = DStarLite(grid, start, goal)
    visited = planner.compute_shortest_path()
    path = planner.extract_path()
    return path, visited


def d_star_lite_steps(grid, start: Tuple[int, int], goal: Tuple[int, int]):
    """Same search as DStarLite.compute_shortest_path(), yielding internal
    state at every vertex expansion. Step-dict shape matches
    path_planning.a_star.search_steps() so it can be dropped into the same
    comparison viewers. One-shot only (no incremental replanning) — for
    that, use the DStarLite class.

    Note the g values here are cost-to-GOAL (this search runs backward from
    goal), the mirror image of the forward searches in a_star.search_steps.
    """
    planner = DStarLite(grid, start, goal)
    t0 = time.perf_counter()
    expanded = 0
    max_frontier = 1

    while planner.open_set and (
        planner.open_set[0][0] < planner._calc_key(planner.start)
        or planner.rhs.get(planner.start, float('inf')) != planner.g.get(planner.start, float('inf'))
    ):
        k_old, u = heapq.heappop(planner.open_set)
        if planner.open_entries.get(u) != k_old:
            continue
        del planner.open_entries[u]
        expanded += 1
        k_new = planner._calc_key(u)
        if k_old < k_new:
            planner._push(u)
        elif planner.g.get(u, float('inf')) > planner.rhs.get(u, float('inf')):
            planner.g[u] = planner.rhs[u]
            for s in planner._neighbors(u):
                planner.update_vertex(s)
        else:
            planner.g[u] = float('inf')
            planner.update_vertex(u)
            for s in planner._neighbors(u):
                planner.update_vertex(s)
        max_frontier = max(max_frontier, len(planner.open_set))
        yield {
            'done': False,
            'mode': 'dstarlite',
            'iteration': expanded,
            'current': u,
            'g': planner.g.get(u, float('inf')),
            'h': heuristic(planner.start, u),
            'f': k_new[0],
            'frontier_nodes': [n for _, n in planner.open_set],
            'expanded': expanded,
            'max_frontier': max_frontier,
        }

    path = planner.extract_path()
    yield {
        'done': True,
        'success': path is not None,
        'mode': 'dstarlite',
        'path': path,
        'path_cost': planner.g.get(start) if path is not None else None,
        'expanded': expanded,
        'max_frontier': max_frontier,
        'elapsed': time.perf_counter() - t0,
    }
