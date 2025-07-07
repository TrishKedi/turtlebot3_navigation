# 🐢 TurtleBot3 Navigation

This project implements a complete **custom navigation system** for a TurtleBot3 robot in ROS (Noetic), using:

- 🧭 Global path planning via **A*** (on an OccupancyGrid map)
- 🤖 **Kinematic waypoint controller** for smooth goal tracking
- 🧱 **Local obstacle avoidance** using **Potential Fields**
- 🎥 RViz and Gazebo for simulation and visualization

All components are implemented **from scratch in Python**, without using the ROS Navigation Stack.

---

## 🧱 Project Structure

```
turtlebot3_navigation/
├── scripts/
│   ├── global_planner.py           # A* path planning
│   ├── kinematic_controller.py     # Waypoint tracking controller
│   ├── potential_fields.py         # Obstacle avoidance
│   └── navigator.py                # Integration of planning and control
├── maps/
│   ├── my_map.pgm
│   └── my_map.yaml
├── rviz/
│   └── navigation.rviz
├── launch/
│   └── navigation.launch
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 🚀 How to Run

### 1. Set TurtleBot3 model
```bash
export TURTLEBOT3_MODEL=burger
```

### 2. Launch the navigation system
```bash
roslaunch turtlebot3_navigation navigation.launch
```

This will:
- Start Gazebo with TurtleBot3
- Load your static map using `map_server`
- Launch your custom navigation pipeline
- Open RViz for visualization

---

## 🧠 Node Architecture

- **`global_planner.py`**: Subscribes to `/map`, runs A*, outputs waypoints
- **`kinematic_controller.py`**: Converts goal → velocity using unicycle control
- **`potential_fields.py`**: Converts LIDAR points → repulsive velocity vector
- **`navigator.py`**: Main node that:
  - Plans path at startup
  - Tracks waypoints
  - Switches to obstacle avoidance if necessary

---

## 🖥️ Visualizations

RViz shows:
- The static map (`/map`)
- Real-time robot pose (`/odom`)
- Laser scans (`/scan`)
- Planned global path (`/planned_path`)
- Coordinate transforms (`TF`)

---

## 📋 Requirements

- ROS Noetic
- `turtlebot3`, `turtlebot3_gazebo`
- `map_server`, `rviz`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`

---

## 📚 Report

A detailed IEEE-style report is included in `/report/TurtleBot3_Navigation_Report.docx`, covering:
- Planning and control strategy
- Trajectory plots and RViz screenshots
- Quantitative performance results

---

## 🙏 Acknowledgments

This project was developed as part of the **Autonomous Systems** module at **Deggendorf Institute of Technology**.

Special thanks to the teaching team for their support and guidance.

---

## 🛠️ Author

**Patricia Atim**  
Autonomous Systems, DIT  

