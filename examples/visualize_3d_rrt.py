import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import matplotlib

if not os.environ.get('MPLBACKEND'):
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import random
from path_planning.rrt_3d_continuous import rrt_3d

def get_cube_verts(x, y, z, size):
    return [
        [(x,y,z), (x+size,y,z), (x+size,y+size,z), (x,y+size,z)],
        [(x,y,z+size), (x+size,y,z+size), (x+size,y+size,z+size), (x,y+size,z+size)],
        [(x,y,z), (x+size,y,z), (x+size,y,z+size), (x,y,z+size)],
        [(x,y+size,z), (x+size,y+size,z), (x+size,y+size,z+size), (x,y+size,z+size)],
        [(x,y,z), (x,y+size,z), (x,y+size,z+size), (x,y,z+size)],
        [(x+size,y,z), (x+size,y+size,z), (x+size,y+size,z+size), (x+size,y,z+size)]
    ]

def plot_3d_rrt():
    bounds = (20.0, 20.0, 20.0)
    # Random obstacles
    obstacles = []
    for _ in range(20):
        x = random.uniform(2, 17)
        y = random.uniform(2, 17)
        z = random.uniform(2, 17)
        size = random.uniform(1, 2)
        obstacles.append((x, y, z, size))
        
    start = (0.0, 0.0, 0.0)
    goal = (19.0, 19.0, 19.0)
    
    path = rrt_3d(start, goal, bounds, obstacles, max_iters=10000, step_size=1.0)
    
    if not path:
        print("No path to plot.")
        return

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot Path
    path = np.array(path)
    ax.plot(path[:, 0], path[:, 1], path[:, 2], label='Path', color='blue', linewidth=2)
    ax.scatter(start[0], start[1], start[2], color='green', s=50, label='Start')
    ax.scatter(goal[0], goal[1], goal[2], color='red', s=50, label='Goal')
    
    # Plot Obstacles (Cubes)
    for cx, cy, cz, size in obstacles:
        verts = get_cube_verts(cx, cy, cz, size)
        cube = Poly3DCollection(verts, color='orange', alpha=0.3)
        ax.add_collection3d(cube)
        
    ax.set_xlim(0, bounds[0])
    ax.set_ylim(0, bounds[1])
    ax.set_zlim(0, bounds[2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    
    plt.savefig('output/rrt_3d_cubes_demo.png')
    print("Plot saved to output/rrt_3d_cubes_demo.png")

if __name__ == "__main__":
    plot_3d_rrt()
