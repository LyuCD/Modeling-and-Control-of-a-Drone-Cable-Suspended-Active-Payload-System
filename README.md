# Modeling-and-Control-of-a-Drone-Cable-Suspended-Active-Payload-System
Project repository for modeling and control of drone cable-suspended payloads using ROS2 and Gazebo.
# Drone Cable-Suspended Active Payload — Modeling & Control

It implements a full UAV–cable–payload pipeline in **ROS 2 + Gazebo**, with two payload-side controllers:

- **PID** (baseline swing suppression)  
- **MPC** (constraint-aware optimal control with CasADi)

The goal is to suppress payload swing while maintaining stable flight and good tracking.

---

## 1. Dependencies

- Ubuntu 22.04 (recommended)  
- **ROS 2 Humble** (or newer)  
- **Gazebo Fortress/Garden**  
- `rviz2`, `teleop-twist-keyboard`  
- Python 3.8+, with `numpy`, `matplotlib`, `casadi`


## 2. Run

### 2.1 Bring up simulation

```bash
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
```

This launches Gazebo, spawns the UAV + payload, opens RViz, and starts teleop.


## 3. Topics

| Topic                  | Type                       | Notes |
|-------------------------|----------------------------|-------|
| `/payload_state`        | `nav_msgs/Odometry` or custom | payload α, β, x, y, vx, vy |
| `/uav/odom`             | `nav_msgs/Odometry`        | UAV pose |
| `/payload_force_cmd`    | `geometry_msgs/Vector3`    | Active force `[fx, fy, 0]` |
| `/cmd_vel`              | `geometry_msgs/Twist`      | Teleop (optional) |

---


## 4. Results

- **No control**: large swings, long recovery  
- **PID**: basic stability with low effort  
- **MPC**: reduced peak swing, shorter settling, improved error indices, smoother forces  

Plots and metrics are saved in `/results`.

---

## 5. Scenarios

- Hover → step move: (0,0) → (1,1)  
- Start/stop acceleration  
- Force-cap sweep (0.5–3.0 N)  

## 6. Data

All experimental and simulation results are stored in the `data/` directory.  
It contains the following subfolders:

- **figs_all/** – all generated figures from simulations (complete set).  
- **figs_by_case/** – results grouped by test cases (e.g., hover, step move, acceleration).  
- **figs_compare/** – comparison plots between different controllers (No Control, PID, MPC).  
- **mpc_figs/** – specific plots and metrics related to the MPC controller.  

These figures are used in the analysis and result sections of the dissertation.

# sjtu_drone
sjtu_drone is a quadrotor simulation program forked from [tum_simulator](http://wiki.ros.org/tum_simulator), developed using ROS + Gazebo.

The acronym 'sjtu' stands for Shanghai Jiao Tong University. This package has been used in the past for testing algorithms for the [UAV contest at SJTU](http://mediasoc.sjtu.edu.cn/wordpress)



