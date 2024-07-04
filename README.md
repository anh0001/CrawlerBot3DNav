# Crawler Robot Project

## Overview
This repository contains the software for a crawler robot designed to navigate uneven terrain. The robot is equipped with tank-like wheels, two motors (left and right), a Livox MID-360 3D LiDAR, and an RGBD RealSense D455 camera. It is controlled remotely via wireless remote and records sensor measurements for later processing, with the goal of achieving future autonomous navigation.

## Getting Started
1. **Clone the repository:**
    ```sh
    git clone https://github.com/anh0001/CrawlerBot3DNav.git
    cd CrawlerBot3DNav
    ```

2. **Setup the environment:**
    - Ensure you have ROS installed. Follow the instructions on the [ROS installation page](http://wiki.ros.org/ROS/Installation).
    - Source the `cmds.sh` script and perform the setup:
        ```sh
        source cmds.sh
        ./cmds.sh setup
        ```

## Contributing
Contributions are welcome! Please fork this repository and submit pull requests for any improvements or bug fixes.

## License
This project is licensed under the MIT License - see the LICENSE file for details.
