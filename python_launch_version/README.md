# ArUco Marker Command-Following Robot (Python Launch Version)

This directory contains the updated, fully debugged, and optimized version of the ArUco command follower robot using **Python Launch Files** (`my_robot.launch.py`).

---

## 🚀 Key Improvements & Fixes

1. **Python Launch System (`my_robot.launch.py`)**:
   - Replaced legacy `.xml` launch files with standard ROS 2 Python launch descriptions.
   - Fixed path parsing shell syntax errors (such as spaces/parentheses in paths, e.g. `trial3_ihub (version 2 Copy)`) by quoting Gazebo arguments securely using Python's `shlex`.

2. **Resolved Duplicate Nodes / RViz**:
   - Explicitly assigned unique node names (`name="rviz2"` and `name="aruco_command_follower"`) to prevent duplicate instances or zombie processes from running in parallel.

3. **Collision Constraint & Footprint Fixes (Robot Stuck / Corner Cutting)**:
   - Configured exact robot polygon dimensions `[[-0.30, -0.22], [-0.30, 0.22], [0.30, 0.22], [0.30, -0.22]]` matching the actual physical size (0.6m length x 0.44m width) instead of a simple 0.22m radius circle.
   - Adjusted `inflation_radius` to `0.55m` and optimized local/global costmaps to prevent the robot from plan-clipping walls or getting stuck in tight gaps.

4. **Corrected Command / Collision Monitor Conflict**:
   - Fixed the `collision_monitor` pipeline. Originally, its direct output to `/cmd_vel` conflicted with the ArUco follower's commands and caused a lifecycle heartbeat timeout/crash when the simulator lagged.
   - Resolved the asynchronous preemption race condition in `aruco_command_follower.py` by removing redundant/manual cancellations, letting Nav2's Action Server naturally preempt goals cleanly without dropping connection or reverting to searching.

---

## 🛠️ How to Build & Run

### 1. Build the Workspace
From the workspace root directory:
```bash
colcon build --packages-select my_robot_bringup my_test_pkg --symlink-install
```

### 2. Source the Setup
```bash
source install/setup.bash
```

### 3. Launch the Robot Stack
Launch the complete simulation environment (Gazebo, SLAM Toolbox, Nav2, RViz, and the ArUco Follower):
```bash
ros2 launch my_robot_bringup my_robot.launch.py headless:=false
```

---

## 📂 Directory Structure

```text
python_launch_version/
├── README.md               # This documentation
└── src/
    ├── my_robot_bringup/   # Configuration files (nav2_params.yaml), Gazebo world, and my_robot.launch.py
    └── my_test_pkg/        # ArUco command follower python node (aruco_command_follower.py)
```
