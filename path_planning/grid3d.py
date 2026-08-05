import numpy as np
from typing import Tuple, List

class Grid3D:
    def __init__(self, width: int, height: int, depth: int, obstacle_prob: float = 0.2, seed: int = None):
        self.width = width
        self.height = height
        self.depth = depth
        rng = np.random.RandomState(seed)
        self.cells = (rng.rand(depth, height, width) < obstacle_prob).astype(np.uint8)

    def in_bounds(self, x: int, y: int, z: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height and 0 <= z < self.depth

    def is_free(self, x: int, y: int, z: int) -> bool:
        if not self.in_bounds(x, y, z):
            return False
        return self.cells[z, y, x] == 0

    def set_free(self, x: int, y: int, z: int):
        if self.in_bounds(x, y, z):
            self.cells[z, y, x] = 0

    def neighbors6(self, x: int, y: int, z: int) -> List[Tuple[int, int, int]]:
        n = []
        for dx, dy, dz in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
            nx, ny, nz = x + dx, y + dy, z + dz
            if self.in_bounds(nx, ny, nz) and self.is_free(nx, ny, nz):
                n.append((nx, ny, nz))
        return n
