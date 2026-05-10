# Ackermann Perception-to-Action

Greenswip / Revati Technologies submission.

ROS 2 Jazzy + Gazebo Harmonic. An Ackermann-steered robot uses a camera to find a red box among 3 decoys (sphere, capsule, cylinder) and drives to it.

📹 [Demo video] : https://drive.google.com/drive/folders/1-RzxqcahzAISCw-p1P6YWTaRtN6bFpO9

## How it works

- Vision node — HSV red filter + 4-corner shape check. Box is the only object that's both red and 4-cornered, so it works regardless of where things are placed.
- Control node — turn-then-drive P-controller. Steers proportionally to the box's pixel offset. Never spins in place (Ackermann constraint).

## Setup
cd ~/ackermann_ws
colcon build --symlink-install
source install/setup.bash

## Run (3 terminals)

Terminal 1
ros2 launch ackermann_bot robot.launch.py

Terminal 2
ros2 run ackermann_bot vision_node.py

Terminal 3
ros2 run ackermann_bot control_node.py

The robot detects the red box and drives to it.

## Test robustness (3 shuffled arrangements)
~/ackermann_ws/scripts/shuffle.sh 1   # default
~/ackermann_ws/scripts/shuffle.sh 2   # box on left
~/ackermann_ws/scripts/shuffle.sh 3   # box far away

After each shuffle, run `control_node.py`. Robot reaches the box every time.

## Files

ackermann_bot/
├── urdf/ack.urdf.xacro    # Robot + Gazebo plugins
├── worlds/shapes.sdf      # 4 colored objects
├── config/bridge.yaml     # ROS-Gazebo topic bridges
├── launch/robot.launch.py # One-command startup
├── scripts/
│   ├── vision_node.py     # OpenCV detection
│   ├── control_node.py    # Ackermann controller
│   └── shuffle.sh         # Robustness test helper
└── notes/debug_log.md     # AI prompts + bugs caught

## Author

Varun Manikonda — MIT Manipal · Cyber Physical Systems
