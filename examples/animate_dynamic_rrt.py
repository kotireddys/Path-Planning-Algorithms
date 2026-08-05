import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import matplotlib

if not os.environ.get('MPLBACKEND'):
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
import io
from path_planning.rrt_dynamic import rrt_dynamic
from path_planning.dynamic_obstacle import DynamicObstacle
from visualize import save_gif

def animate_dynamic_rrt():
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
        print("No path found.")
        return

    frames = []
    times = [p[5] for p in path]
    max_t = max(times)
    
    # Generate frames
    for current_t in np.linspace(0, max_t, 30):
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot path up to current time
        past_pts = np.array([p[:3] for p in path if p[5] <= current_t])
        if len(past_pts) > 0:
            ax.plot(past_pts[:, 0], past_pts[:, 1], past_pts[:, 2], color='blue', linewidth=2)
            ax.scatter(past_pts[-1, 0], past_pts[-1, 1], past_pts[-1, 2], color='blue', s=50) # Drone pos
            
        # Plot obstacles at current time
        for obs in obstacles:
            pos = obs.get_pos(current_t)
            ax.scatter(pos[0]+obs.size/2, pos[1]+obs.size/2, pos[2]+obs.size/2, 
                       color='orange', alpha=0.6, s=obs.size**3 * 20, marker='s')
        
        ax.set_xlim(0, bounds[0])
        ax.set_ylim(0, bounds[1])
        ax.set_zlim(0, bounds[2])
        ax.set_title(f"Time: {current_t:.1f}")
        
        fig.canvas.draw()
        buf = fig.canvas.buffer_rgba()
        w, h = fig.canvas.get_width_height()
        image = np.frombuffer(buf, dtype=np.uint8).reshape((h, w, 4))[..., :3]
        frames.append(image)
        plt.close(fig)
        
    save_gif(frames, 'output/rrt_dynamic_animation.gif', fps=5)
    print("Animation saved to output/rrt_dynamic_animation.gif")

if __name__ == "__main__":
    animate_dynamic_rrt()
