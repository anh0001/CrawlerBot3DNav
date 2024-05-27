# Repository Structure
## 1
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

## 2 with web interface
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
│   ├── visualization.launch
│   └── web_interface.launch           # Add this file
│
├── src/
│   ├── robot_control/
│   ├── src/
│   │   ├── __init__.py
│   │   ├── motor_control.py
│   │   ├── remote_control.py
│   │   └── recording_control.py      # Add this file
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
├── web_interface/                    # Add this directory
│   ├── index.html                    # Add this file
│   ├── css/
│   │   └── styles.css                # Add this file (if needed)
│   └── js/
│       ├── rosbridge_setup.js        # Add this file
│       └── sensor_visualization.js   # Add this file
│
├── scripts/
│   ├── start_robot.sh
│   ├── stop_robot.sh
│   ├── record_data.sh
│   └── start_web_server.sh           # Add this file
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