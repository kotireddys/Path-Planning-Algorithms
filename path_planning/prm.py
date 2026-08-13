import heapq
import random
from typing import Tuple, List, Dict, Optional

from .rrt import distance, collision_free


def build_roadmap(grid, num_samples: int = 200, connect_radius: float = 12.0, seed=None):
    """Sample num_samples random free points and connect any pair within
    connect_radius whose straight-line segment is collision-free.

    Returns (nodes, edges): nodes is a list of (x, y) points, edges is an
    adjacency dict node_index -> [(neighbor_index, cost), ...].
    """
    rng = random.Random(seed)
    nodes: List[Tuple[float, float]] = []
    attempts = 0
    while len(nodes) < num_samples and attempts < num_samples * 20:
        attempts += 1
        x = rng.uniform(0, grid.width - 1)
        y = rng.uniform(0, grid.height - 1)
        if grid.is_free(int(round(x)), int(round(y))):
            nodes.append((x, y))

    edges: Dict[int, List[Tuple[int, float]]] = {i: [] for i in range(len(nodes))}
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            d = distance(nodes[i], nodes[j])
            if d <= connect_radius and collision_free(grid, nodes[i], nodes[j]):
                edges[i].append((j, d))
                edges[j].append((i, d))
    return nodes, edges


def _splice_in(grid, nodes, edges, pt, connect_radius):
    """Add pt as a new roadmap node, wiring it to every existing node it
    can reach in a straight line within connect_radius. Used to connect
    start/goal into a prebuilt roadmap."""
    idx = len(nodes)
    nodes.append(pt)
    edges[idx] = []
    for i, n in enumerate(nodes[:idx]):
        d = distance(n, pt)
        if d <= connect_radius and collision_free(grid, n, pt):
            edges[idx].append((i, d))
            edges[i].append((idx, d))
    return idx


def _dijkstra_over_roadmap(nodes, edges, start_idx, goal_idx):
    dist = {start_idx: 0.0}
    came_from: Dict[int, int] = {}
    visited_order = []
    open_set = [(0.0, start_idx)]
    closed = set()
    while open_set:
        d, u = heapq.heappop(open_set)
        if u in closed:
            continue
        closed.add(u)
        visited_order.append(u)
        if u == goal_idx:
            path = [u]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            return [nodes[i] for i in path], visited_order
        for v, w in edges.get(u, []):
            nd = d + w
            if nd < dist.get(v, float('inf')):
                dist[v] = nd
                came_from[v] = u
                heapq.heappush(open_set, (nd, v))
    return None, visited_order


def prm(grid, start: Tuple[int, int], goal: Tuple[int, int], num_samples: int = 200,
        connect_radius: float = 12.0, seed=None):
    """Probabilistic Roadmap: sample free space once to build a graph, then
    shortest-path over it with Dijkstra. Unlike RRT (a single tree grown
    toward the goal), the roadmap is reusable — the same graph could answer
    many different start/goal queries. Here we build-and-query in one call.

    Returns (path_pts, nodes, edges, visited_order) — nodes/edges expose the
    full roadmap for visualization, mirroring rrt()'s (nodes, parent).
    """
    nodes, edges = build_roadmap(grid, num_samples=num_samples, connect_radius=connect_radius, seed=seed)
    start_idx = _splice_in(grid, nodes, edges, start, connect_radius)
    goal_idx = _splice_in(grid, nodes, edges, goal, connect_radius)
    path_pts, visited_order = _dijkstra_over_roadmap(nodes, edges, start_idx, goal_idx)
    return path_pts, nodes, edges, visited_order
