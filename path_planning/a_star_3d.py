import heapq
from typing import Tuple, List, Dict

def heuristic(a: Tuple[int, int, int], b: Tuple[int, int, int]) -> float:
    # Manhattan distance in 3D
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])

def astar_3d(grid, start: Tuple[int, int, int], goal: Tuple[int, int, int]):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}
    g_score = {start: 0}
    visited_order = []

    while open_set:
        _, current = heapq.heappop(open_set)
        visited_order.append(current)
        if current == goal:
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            return path, visited_order

        for neighbor in grid.neighbors6(*current):
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, neighbor))

    return None, visited_order
