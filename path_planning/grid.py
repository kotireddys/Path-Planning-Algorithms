import numpy as np
from typing import Tuple, List


class Grid:
    def __init__(self, width: int, height: int, obstacle_prob: float = 0.2, seed: int = None):
        self.width = width
        self.height = height
        rng = np.random.RandomState(seed)
        self.cells = (rng.rand(height, width) < obstacle_prob).astype(np.uint8)

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, x: int, y: int) -> bool:
        if not self.in_bounds(x, y):
            return False
        return self.cells[y, x] == 0

    def set_free(self, x: int, y: int):
        if self.in_bounds(x, y):
            self.cells[y, x] = 0

    def neighbors4(self, x: int, y: int) -> List[Tuple[int, int]]:
        n = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if self.in_bounds(nx, ny) and self.is_free(nx, ny):
                n.append((nx, ny))
        return n
