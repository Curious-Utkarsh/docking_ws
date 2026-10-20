# Autonomous Docking with ArUco Markers — TurtleBot3, ROS 2

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Nav2](https://img.shields.io/badge/Nav2-navigation-1F6FEB)](https://docs.nav2.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-ArUco-5C3EE8?logo=opencv)](https://opencv.org/)

A TurtleBot3 that drives itself to a charging dock when its battery runs low. Nav2 takes the robot to the dock area while avoiding obstacles; ArUco markers detected with OpenCV guide the final alignment. If the main dock is unavailable, the robot reroutes to an alternative dock.

## Demo

<p align="center">
  <a href="https://youtu.be/WPieODtJKek"><img src="https://img.youtube.com/vi/WPieODtJKek/hqdefault.jpg" width="420" alt="TurtleBot3 autonomous docking with ArUco markers"></a>
  <br><sub>Click to watch · <a href="https://www.linkedin.com/posts/kumar-utkarsh-124aab227_robotics-ros2-automation-activity-7262365107433725954-rZ_E">LinkedIn post</a></sub>
</p>

More TurtleBot3 + Nav2 runs from this workspace: [maze solver](https://youtu.be/Dcz1byU75Pg) · [house patrolling](https://youtu.be/aSTW4Spc6-M)

## Key features

- **Battery-triggered docking** — subscribes to `/battery`; at ≤ 20 % it sends a Nav2 goal to the dock.
- **Marker-based precision** — ArUco markers are detected in the camera image and published as TF frames for the final alignment.
- **Fallback docking** — if the main dock is blocked or unavailable, the robot redirects to an alternate dock.
- **Dynamic obstacle avoidance** — Nav2 replans around obstacles on the way.

## Software stack

ROS 2 Humble · Nav2 · TurtleBot3 (waffle) · Gazebo Classic · OpenCV (ArUco) · tf2 · Python

## Run

```bash
# prerequisites (Ubuntu 22.04, ROS 2 Humble)
sudo apt install ros-humble-turtlebot3* ros-humble-nav2-bringup ros-humble-gazebo-ros-pkgs
pip3 install opencv-contrib-python==4.7.0.72 numpy==1.21.5
export TURTLEBOT3_MODEL=waffle

# workspace
mkdir -p ~/dock_ws && cd ~/dock_ws
git clone https://github.com/Curious-Utkarsh/TurtleBot3_Autonomous_Docking.git .
colcon build && source install/setup.bash

# terminals 1–5
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
rviz2 -d src/turtlebot3_gazebo/rviz/config.rviz
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=src/turtlebot3_gazebo/maps/my_house.yaml
ros2 run bot_script pose_estimator
ros2 run bot_script navigation

# trigger docking: battery 10 % (threshold is 20 %)
ros2 topic pub /battery std_msgs/msg/Int64 "data: 10"
```

<details>
<summary><b>Docking logic and troubleshooting</b></summary>

```python
def batteryCallback(self, status):
    if status.data <= 20 and self.is_docking == False:
        self.is_docking = True
        self.set_and_follow_goal(-0.6, 5.2, -0.1)  # main docking position
    if status.data > 20:
        self.is_docking = False
```

- `src/turtlebot3_gazebo/` — worlds, models, RViz config and the `my_house` map
- `src/bot_script/` — `pose_estimator` (marker detection → TF) and `navigation` (battery monitoring, Nav2 goals, docking)
- Gazebo/RViz out of sync → adjust the simulation rate; missing camera or LiDAR topics → check the URDF plugins; keep `use_sim_time` true for all nodes; tune `nav2_params.yaml` if the robot hesitates near obstacles.

</details>

## Related

This was my first docking project. The idea grew into vision docking on [Aura](https://github.com/Curious-Utkarsh/ROS2_Mobile_Manipulator_Gazebo), autonomous undocking on [Mobi](https://github.com/Curious-Utkarsh/Indoor_Service_AMR), and QR + LiDAR docking on real hardware with [MechaBot](https://github.com/MechaMind-Labs/AMR-Mastery).

---
[Kumar Utkarsh](https://github.com/Curious-Utkarsh) · [LinkedIn](https://www.linkedin.com/in/kumar-utkarsh-124aab227/) · [MechaMind Labs](https://www.mechamindlabs.com/)
