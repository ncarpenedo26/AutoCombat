#!/bin/bash
set -e

# ROS environment already sourced globally in Dockerfile

echo "[postCreateCommand] Updating rosdep..."
rosdep update

echo "[postCreateCommand] Installing dependencies..."
rosdep install --from-paths src --ignore-src -y --skip-keys="rti-connext-dds-6.0.1 fastcdr"

echo "[postCreateCommand] Done."
