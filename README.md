# Turn Controller for Robot Control using ROSBot XL

## Objective: 
- Task 3: Write a PID controller to turn the robot in the XY plane
- Task 4: Test the PID Controller in the Cyberworld real robot lab
    
## Overview
This package implements a PID controller for precise in-place turning of the ROSBot XL robot in a maze environment. The controller guides the robot through a series of predefined angular waypoints, ensuring accurate orientation changes without overshooting.

## Features
- PID-controlled angular velocity for precise turns
- Handles multiple waypoints sequentially
- Optimized for the ROSBot XL's inertial characteristics
- Works in the Gazebo maze simulation environment

## Requirements
- ROS 2 Humble
- ROSBot XL simulation packages
- Gazebo
- `teleop_twist_keyboard` (for manual waypoint measurement)

## Installation
1. Clone this package to your ROS 2 workspace `src` directory
    ```bash
    mkdir -p ~/ros2_ws/src
    git clone https://github.com/kailash197/cp17_turn_controller.git turn_controller
    ```
2. Build the package:
   ```bash
   cd ~/ros2_ws 
   colcon build --packages-select turn_controller 
   source install/setup.bash
   ```
## Usage (Task3: Simulation)
1. Start the simulation:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch rosbot_xl_gazebo simulation.launch.py
   ```
2. Run the turn controller:
    ```bash
    cd ~/ros2_ws/src/turn_controller
    git checkout task3
    cd ~/ros2_ws && colcon build && source install/setup.bash
    source ~/ros2_ws/install/setup.bash && ros2 run turn_controller turn_controller
    ```

## Usage (Task4: Real Robot)
1. Connect to real robot

2. Run the turn controller:
    ```bash
    cd ~/ros2_ws/src/turn_controller
    git checkout task4
    cd ~/ros2_ws && colcon build && source install/setup.bash
    source ~/ros2_ws/install/setup.bash && ros2 run turn_controller turn_controller 2
   ```

## Useful Commands:

#### View orientation
```bash
ros2 topic echo /rosbot_xl_base_controller/odom --field pose.pose.orientation
```

#### Create tags
```bash
git tag <tagname> <commit-id>
```

#### View tags
```bash
git tag
git ls-remote --tags origin # remote tags
```

#### Push tags
```bash
git push origin --tags
```

#### Remove tags
```bash
git tag -d <tagname>
git push origin --delete <tagname>  # Delete remotely
```
