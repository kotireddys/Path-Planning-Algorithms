import numpy as np

class DynamicObstacle:
    def __init__(self, pos, vel, size):
        self.pos_0 = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)
        self.size = size

    def get_pos(self, t):
        return self.pos_0 + self.vel * t

    def is_colliding(self, point, t):
        pos = self.get_pos(t)
        # Check if point is inside AABB at time t
        return (pos[0] <= point[0] <= pos[0] + self.size and
                pos[1] <= point[1] <= pos[1] + self.size and
                pos[2] <= point[2] <= pos[2] + self.size)
