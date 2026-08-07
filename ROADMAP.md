# Path-Planning-Algorithms — Roadmap

This repository is organized as a three-phase portfolio to demonstrate path
planning algorithms and then integrate them into simulated robotics workflows.

Phase 1 — 2D Path Planning Sandbox (current)
- Implement A*, RRT, RRT*, and Dubins path from scratch in Python.
- Create a 2D grid environment with random obstacles.
- Provide visualization (matplotlib) and animated GIFs for each algorithm.
- Deliverable: `Path-Planning-Algorithms` repository with demos and GIFs.

Phase 2 — Connect to 3D Simulator (PX4 + Gazebo)
- Provide scripts and Dockerfile to run PX4 SITL and bridge telemetry to ROS 2.
- Load a quadcopter in Gazebo and write ROS 2 nodes for control.
- Deliverable: `ROS2-Drone-Controller` repository with instructions and recordings.

Phase 3 — SLAM & Collision Avoidance (Full stack)
- Attach LiDAR / RGB-D sensors in simulation and run SLAM to build occupancy maps.
- Feed the occupancy grid into the RRT* planner for online re-planning.
- Deliverable: `Autonomous-Drone-SLAM-Navigation` repository with demos and videos.

Next immediate steps
- Finish RRT visualization and implement RRT* and Dubins path planning.
- Produce GIFs and update `README.md` with visual proof.
