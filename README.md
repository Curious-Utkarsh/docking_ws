# Autonomous Docking System Using ArUco Markers in ROS2

This repository provides a ROS2-based solution for an autonomous docking system that leverages ArUco markers for marker-based navigation and docking. Using turtlebot3_gazebo, nav2, and opencv, this solution enables the robot to autonomously navigate to a docking station upon receiving a low-battery signal while dynamically avoiding obstacles.

Prerequisites
System Requirements

    ROS2: Humble Hawksbill
    Gazebo: Gazebo Classic (turtlebot3_gazebo requires it)
    Python 3 with pip
    sudo apt install python3-pip

    OpenCV and other Python packages


Step 1: ROS2 Installation

    Set up ROS2 Humble: Follow the official ROS2 Humble installation guide.

    Source ROS2 Environment:

    source /opt/ros/humble/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    export TURTLEBOT3_MODEL=waffle
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export PATH=$PATH:~/.local/bin 
    source /usr/share/gazebo-11/setup.bash
    export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
    export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
    export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11

Step 2: Install Gazebo and ROS2 Control Packages

    Install Gazebo and Control Packages:

    sudo apt update
    sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control

    Install Additional Gazebo ROS Packages:

    sudo apt-get install ros-humble-gazebo-ros-pkgs

Step 3: Set up Repo

    Clone the TurtleBot3 Gazebo Repository:
    
    mkdir docking_ws
    cd ~/dock_ws
    git clone https://github.com/Curious-Utkarsh/docking_ws.git

    Install TurtleBot3 Packages:
    
    sudo apt install ros-humble-turtlebot3* 
    
    Set TurtleBot3 Model:

    export TURTLEBOT3_MODEL=waffle

Step 4: Install Python Dependencies

    Install OpenCV and Numpy:

    pip3 install opencv-contrib-python==4.7.0.72 numpy==1.21.5

    Install Other Dependencies (if any): List other dependencies if applicable.

    Build the Workspace:

    cd ~/dock_ws
    colcon build
    source install/setup.bash

    Verify Installation: Ensure all dependencies are correctly installed and sourced.

Running the Simulation and Docking System

To run the autonomous docking system, follow these steps:

    Launch the Gazebo World with TurtleBot3:

    ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

Open RViz with Configuration:

    rviz2 -d dock_ws/src/turtlebot3_gazebo/rviz/config.rviz

Run the Nav2 Bringup for Navigation and Localization:

    ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=dock_ws/src/turtlebot3_gazebo/maps/my_house.yaml

Start the Pose Estimator Node: This node is responsible for estimating the robot's pose based on marker detection.

    ros2 run bot_script pose_estimator

Run the Navigation Node: This node performs the navigation and docking logic, using ArUco markers for precise docking.

    ros2 run bot_script navigation

File Structure

    turtlebot3_gazebo/: Contains Gazebo worlds, models, and RViz configuration.
    bot_script/: Includes custom nodes for pose estimation (pose_estimator) and navigation (navigation).
    maps/: Contains the my_house.yaml map file for localization.

Troubleshooting

    Gazebo and RViz Synchronization Issues: If there is a delay between Gazebo and RViz, adjust the simulation rate in the Gazebo parameters.
    Missing Camera or LiDAR Topics: Ensure that all required plugins are loaded in the URDF model.
    Low-Battery Simulation: Manually adjust the battery topic to simulate a low-battery signal for testing the autonomous docking behavior.

Additional Notes

    Simulation Time: Ensure that use_sim_time is enabled for all nodes.
    Parameter Tuning: Adjust parameters in nav2_params.yaml to refine navigation, particularly if there are obstacles.

Acknowledgments

    TurtleBot3 Simulation Packages by ROBOTIS
    ROS2 and Nav2 Documentation
    OpenCV for ArUco Marker Detection

License

This project is licensed under the MIT License. See the LICENSE file for more details.
