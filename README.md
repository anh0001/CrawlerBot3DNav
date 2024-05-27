
# Crawler Robot Project

## Overview
This repository contains the software for a crawler robot designed to navigate uneven terrain. The robot is equipped with tank-like wheels, two motors (left and right), a 3D LiDAR, and an RGBD RealSense camera. It is controlled remotely via wireless remote and records sensor measurements for later processing, with the goal of achieving future autonomous navigation.

## Getting Started
1. **Clone the repository:**
    ```
    git clone https://github.com/yourusername/crawler_robot_project.git
    cd crawler_robot_project
    ```

2. **Setup the environment:**
    - Ensure you have ROS installed. Follow the instructions on the [ROS installation page](http://wiki.ros.org/ROS/Installation).
    - Install necessary Python packages:
        ```
        pip install -r requirements.txt
        ```

3. **Build the workspace:**
    ```
    catkin_make
    source devel/setup.bash
    ```

4. **Launch the robot:**
    ```
    roslaunch launch/robot_bringup.launch
    ```

## Contributing
Contributions are welcome! Please fork this repository and submit pull requests for any improvements or bug fixes.

## License
This project is licensed under the MIT License - see the LICENSE file for details.
