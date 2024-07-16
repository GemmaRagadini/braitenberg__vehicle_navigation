# Braitenberg Vehicle Navigation

## Project Overview

The `husky_follow_light` package has been developed to enable a robot to navigate a maze using reactive control based on robot camera images.

## Package Contents

The package includes the following components:
- **Nodes**:
  - `image_processing_node.py`
  - `motor_control_node.py`
  - `lights_switcher.py` (created by professors,for changing the light colors)
- **Launch File**:
  - `navigate_maze.launch` (to start the simulation)

## Directory Structure

braitenberg_nav_project/ROS_Plugins/catkin_ws/src/husky_follow_light\
│\
├── nodes\
│ ├── image_processing_node.py\
│ ├── motor_control_node.py\
│ └── lights_switcher.py\
│\
├── launch\
│ ├── navigate_maze.launch

## How to Run the Simulation

1. **Source the setup script**:

   ```bash
   source ./ROS_Plugins/catkin_ws/devel/setup.bash
1. **Launch the simulation**

   ```bash
    roslaunch husky_follow_light navigate_maze.launch
