import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import matplotlib

if not os.environ.get('MPLBACKEND'):
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from path_planning.rrt_dynamic import rrt_dynamic
from path_planning.dynamic_obstacle import DynamicObstacle

def plot_dynamic_rrt():
    bounds = (20.0, 20.0, 20.0)
    obstacles = [
        DynamicObstacle(pos=(10, 5, 5), vel=(0, 0.5, 0), size=2),
        DynamicObstacle(pos=(5, 15, 10), vel=(0.5, -0.5, 0), size=2),
        DynamicObstacle(pos=(15, 10, 15), vel=(-0.5, 0, -0.5), size=2)
    ]
    start = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    goal = (19.0, 19.0, 19.0)
    
    path = rrt_dynamic(start, goal, bounds, obstacles, velocity=1.0)
    
    if not path:
        print("No path to plot.")
        return

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot Path
    path_pts = np.array([p[:3] for p in path])
    ax.plot(path_pts[:, 0], path_pts[:, 1], path_pts[:, 2], label='Path', color='blue', linewidth=2)
    ax.scatter(start[0], start[1], start[2], color='green', s=50, label='Start')
    ax.scatter(goal[0], goal[1], goal[2], color='red', s=50, label='Goal')
    
    # Plot Obstacles (at t=0, t=mid, t=end)
    times = [0, max([p[5] for p in path])/2, max([p[5] for p in path])]
    colors = ['orange', 'yellow', 'red']
    for t, color in zip(times, colors):
        for obs in obstacles:
            pos = obs.get_pos(t)
            # Simple marker for cube
            ax.scatter(pos[0]+obs.size/2, pos[1]+obs.size/2, pos[2]+obs.size/2, 
                       color=color, alpha=0.3, s=obs.size**3 * 50, marker='s')
        
    ax.set_xlim(0, bounds[0])
    ax.set_ylim(0, bounds[1])
    ax.set_zlim(0, bounds[2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    
    plt.savefig('output/rrt_dynamic_demo.png')
    print("Plot saved to output/rrt_dynamic_demo.png")

if __name__ == "__main__":
    plot_dynamic_rrt()
