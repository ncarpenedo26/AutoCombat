#!/bin/bash

set -e

# echo "📦 Checking for leo_rover packages..."

# cd "$(dirname "$0")"

# mkdir -p src
# cd src

# # if [ ! -d "leo_simulator-ros2" ]; then
# #   echo "Cloning leo_simulator-ros2..."
# #   git clone -b humble https://github.com/LeoRover/leo_simulator-ros2.git
# # else
# #   echo "leo_simulator-ros2 already exists, skipping..."
# # fi

# # if [ ! -d "leo_common-ros2" ]; then
# #   echo "Cloning leo_common-ros2..."
# #   git clone https://github.com/LeoRover/leo_common-ros2.git
# # else
# #   echo "leo_common-ros2 already exists, skipping..."
# # fi

# cd ..

echo "🧼 Updating rosdep..."
rosdep update

echo "🔧 Installing ROS package dependencies..."
rosdep install --from-paths src --ignore-src -r -y

echo "🐍 Installing Python dependencies..."
pip install -r requirements.txt

echo "🔨 Building the workspace..."
colcon build

echo "✅ Setup complete!"
echo "To start working, run:"
echo "source install/setup.bash"