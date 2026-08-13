import heapq
import time
from typing import Tuple, List, Dict


def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def search_steps(grid, start: Tuple[int, int], goal: Tuple[int, int], mode: str = 'astar'):
    """Best-first search over the grid, yielding internal state at every
    expansion instead of only returning the final path. Each yielded dict
    has done=False during the search and done=True on the final step.

    `mode` controls what the priority queue sorts on — same neighbor
    expansion and g-cost bookkeeping in all three, only the priority differs:
      'astar'    priority = g + h  (optimal, directed by the heuristic)
      'dijkstra' priority = g      (optimal, expands uniformly outward)
      'greedy'   priority = h      (fast, but not guaranteed shortest)
    """
    def priority(g, h):
        if mode == 'dijkstra':
            return g
        if mode == 'greedy':
            return h
        return g + h

    open_set = []
    heapq.heappush(open_set, (priority(0, heuristic(start, goal)), start))
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    g_score = {start: 0}
    visited_order = []
    max_frontier = 1
    t0 = time.perf_counter()

    while open_set:
        f, current = heapq.heappop(open_set)
        visited_order.append(current)
        g = g_score[current]
        h = heuristic(current, goal)
        yield {
            'done': False,
            'mode': mode,
            'iteration': len(visited_order),
            'current': current,
            'g': g,
            'h': h,
            'f': f,
            'frontier_nodes': [node for _, node in open_set],
            'expanded': len(visited_order),
            'max_frontier': max_frontier,
        }
        if current == goal:
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            yield {
                'done': True, 'success': True, 'mode': mode, 'path': path,
                'path_cost': g_score[goal], 'expanded': len(visited_order),
                'max_frontier': max_frontier, 'elapsed': time.perf_counter() - t0,
            }
            return

        for neighbor in grid.neighbors4(*current):
            tentative_g = g + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                nh = heuristic(neighbor, goal)
                heapq.heappush(open_set, (priority(tentative_g, nh), neighbor))
        max_frontier = max(max_frontier, len(open_set))

    yield {
        'done': True, 'success': False, 'mode': mode, 'path': None,
        'expanded': len(visited_order), 'max_frontier': max_frontier,
        'elapsed': time.perf_counter() - t0,
    }


def astar_steps(grid, start: Tuple[int, int], goal: Tuple[int, int]):
    """Same algorithm as astar(), but yields the internal state at every
    expansion. Thin wrapper over search_steps(mode='astar')."""
    yield from search_steps(grid, start, goal, mode='astar')


def astar(grid, start: Tuple[int, int], goal: Tuple[int, int]):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    g_score = {start: 0}
    visited_order = []

    while open_set:
        _, current = heapq.heappop(open_set)
        visited_order.append(current)
        if current == goal:
            # reconstruct
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            return path, visited_order

        for neighbor in grid.neighbors4(*current):
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, neighbor))

    return None, visited_order
