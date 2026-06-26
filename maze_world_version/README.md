# ArUco Marker Command-Following Robot (New Maze World Version)

This directory contains the updated version of the ArUco command follower robot running in the **New Maze World** environment using **Python Launch Files** (`my_robot.launch.py`).

---

## 🚀 Key Features & Updates

1. **New Maze World (`my_world.sdf`)**:
   - The Gazebo simulation features a more complex maze layout.
   - The robot starts positioned outside the maze entrance, executing an initial entry sequence before transitioning to active ArUco marker scanning.
   - Waypoints and marker positions have been optimized for better visibility and navigation path planning within the maze.

2. **Python Launch System (`my_robot.launch.py`)**:
   - Built on ROS 2 Python Launch definitions to coordinate Gazebo Sim, SLAM Toolbox, Nav2, RViz, and the ArUco follower.
   - Implements robust path parsing that handles spaces and special characters in paths safely.

3. **Collision & Navigation Tuning**:
   - Configured exact robot polygon footprint boundaries `[[-0.30, -0.20], [-0.30, 0.20], [0.30, 0.20], [0.30, -0.20]]` to prevent the robot from clipping corners or getting stuck.
   - Optimized inflation layers and local/global planner parameters in `nav2_params.yaml`.

---

## 🛠️ How to Build & Run

### 1. Build the Workspace
From the workspace root directory:
```bash
colcon build --symlink-install
```

### 2. Source the Setup
```bash
source install/setup.bash
```

### 3. Launch the Simulation
Launch the new maze world simulation along with navigation and the command follower:
```bash
ros2 launch my_robot_bringup my_robot.launch.py headless:=false
```

---

## 📂 Directory Structure

```text
maze_world_version/
├── README.md               # This documentation
└── src/
    ├── my_robot_bringup/   # Configs, launch files, and the new maze world.sdf
    ├── my_robot_description/ # URDF, Xacro models, and RViz configs
    ├── my_test_pkg/        # ArUco state machine command follower Python node
    └── turtlesim_mapper/   # Auxiliary plotting utilities
```
