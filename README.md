
# Crawler Robot Project

## Overview
This repository contains the software for a crawler robot designed to navigate uneven terrain. The robot is equipped with tank-like wheels, two motors (left and right), a 3D LiDAR, and an RGBD RealSense camera. It is controlled remotely via wireless remote and records sensor measurements for later processing, with the goal of achieving future autonomous navigation.

## Repository Structure
```
crawler_robot_project/
│
├── README.md
├── LICENSE
├── .gitignore
│
├── docs/
│   ├── design_docs/
│   ├── user_manual/
│   ├── datasheets/
│   └── test_plans/
│
├── config/
│   ├── robot_description/
│   ├── navigation/
│   └── sensors/
│
├── launch/
│   ├── robot_bringup.launch
│   ├── navigation.launch
│   ├── sensor_record.launch
│   └── visualization.launch
│
├── src/
│   ├── robot_control/
│   │   ├── __init__.py
│   │   ├── motor_control.py
│   │   └── remote_control.py
│   │
│   ├── sensor_processing/
│   │   ├── __init__.py
│   │   ├── lidar_processing.py
│   │   ├── realsense_processing.py
│   │   └── data_saving.py
│   │
│   ├── navigation/
│   │   ├── __init__.py
│   │   ├── path_planning.py
│   │   └── obstacle_avoidance.py
│   │
│   └── utils/
│       ├── __init__.py
│       ├── logger.py
│       └── config_loader.py
│
├── scripts/
│   ├── start_robot.sh
│   ├── stop_robot.sh
│   └── record_data.sh
│
├── msgs/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── msg/
│   ├── srv/
│   └── action/
│
├── urdf/
│   ├── robot.urdf
│   └── robot_description.xacro
│
├── worlds/
│   ├── construction_site.world
│   └── test_area.world
│
├── matlab/
│   ├── data_processing/
│   │   ├── read_rosbag.m
│   │   ├── process_lidar_data.m
│   │   ├── process_realsense_data.m
│   │   └── analyze_navigation.m
│   │
│   ├── visualization/
│   │   ├── plot_lidar.m
│   │   ├── plot_camera.m
│   │   └── plot_navigation.m
│   │
│   └── utils/
│       ├── load_config.m
│       ├── save_data.m
│       └── generate_reports.m
│
├── rosbag/
│   └── README.md
│
└── tests/
    ├── integration/
    │   ├── test_navigation.py
    │   ├── test_sensors.py
    │   └── test_control.py
    │
    ├── unit/
    │   ├── test_motor_control.py
    │   ├── test_lidar_processing.py
    │   └── test_path_planning.py
    │
    └── README.md
```

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
