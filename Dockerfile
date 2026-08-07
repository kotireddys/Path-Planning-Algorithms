# Minimal developer Dockerfile for Path-Planning-Algorithms
# This is a lightweight Python container. For Phase 2 (ROS2 / PX4 / Gazebo)
# create a derived image or use a more feature-complete base (see README).

FROM python:3.11-slim

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git ca-certificates wget curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY requirements.txt /workspace/requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

COPY . /workspace

CMD ["bash"]
