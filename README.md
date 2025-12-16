# 🚀 Robust Path Planning for the Andino Robot Using A* Algorithm

**Author:** Luel A. 
📧 luelgeb@uef.fi  

---

## 📌 Overview

This project presents a **robust global path planning solution** for the **Andino mobile robot**, implemented in **ROS 2** using the **A\*** search algorithm.

The default straight-line planner was replaced with a **custom A\*** planner capable of generating **collision-free, smooth paths** on a grid-based occupancy map. Compared to the original planner, this approach enables reliable navigation in **cluttered environments with obstacles**, significantly improving path quality and execution stability.

---

## 🎥 Video Demonstration

📌 *Click the thumbnail below to watch the full simulation demo.*

[![Andino Robot A* Path Planning Demo](https://img.youtube.com/vi/YOUR_VIDEO_ID/0.jpg)](https://www.youtube.com/watch?v=YOUR_VIDEO_ID)

**The video demonstrates:**
- Occupancy grid map loading
- A* path planning in RViz
- Obstacle-aware navigation
- Smooth robot motion in Gazebo
- Terminal output of interpolated waypoints

---

## 🧠 Key Features

✔ Custom **A\*** global planner (replacing `NavfnPlanner`)  
✔ Grid-based planning using `OccupancyGrid`  
✔ Obstacle avoidance with **implicit safety margin**  
✔ Smooth waypoint interpolation (0.2 m spacing)  
✔ Fully integrated into the **ROS 2 navigation framework**  
✔ Validated in **RViz + Gazebo simulation**

---

## 🧭 Methodology

The planner is implemented as a **custom global planner** within ROS 2. It subscribes to the global occupancy grid map (`/map`) and provides a `create_plan` service to generate paths on demand.

### 🔹 A* Path Planning

A* evaluates nodes using:

\[
f(n) = g(n) + h(n)
\]

where:
- \( g(n) \) is the cost from the start
- \( h(n) \) is the heuristic estimate to the goal (Euclidean distance)

An **8-connected grid** is used, enabling diagonal motion and efficient exploration.

---

## 🛡️ Obstacle Avoidance with Safety Margin

Instead of validating only the target grid cell, the planner checks the **8-connected neighborhood** of each cell.  
This creates an **implicit safety buffer**, ensuring that paths do not pass too close to obstacles or unknown regions.

### 🔍 Example: Path turning away from an obstacle
![Obstacle avoidance](media/obstacle_turning.gif)

---

## ✨ Path Smoothing via Waypoint Interpolation

The raw A* output is grid-based and may contain sharp turns.  
To improve motion quality:

- Linear interpolation is applied between consecutive grid cells
- Waypoints are spaced at approximately **0.2 m**
- The interpolated sequence is logged in the terminal

### 🧾 Terminal Output Example
![Terminal output](media/terminal_waypoints.gif)

This results in **smooth, stable trajectories** suitable for real robot execution.

---

## 🧪 Experimental Results

Experiments were conducted in **Gazebo simulation**, with visualization in **RViz**.

### ✅ Observations
- The robot consistently avoided static obstacles
- Alternative routes were found when direct paths were blocked
- Path length decreased smoothly as the robot approached the goal
- No collisions were observed
- Motion execution was smooth and reliable

### 🖼️ Combined RViz + Gazebo View
![RViz and Gazebo](media/rviz_gazebo.gif)

---

## 📂 Repository Structure

---

## ⚙️ How to Run

```bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch andino_custom andino_gz.launch.py



# Robust Path Planning for the Andino Robot Using A* Algorithm

## 🎥 Video Demonstration

![Andino A* Path Planning Demo](media/preview.gif)

▶ **Full simulation video (WEBM):**  
[Click here to watch](media/andino_astar_demo.webm)

