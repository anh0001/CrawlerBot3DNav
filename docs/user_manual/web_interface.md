# Installing ROS Bridge in ROS Noetic on Ubuntu 20

To install ROS Bridge in ROS Noetic on Ubuntu 20, follow these steps:

1. Open a terminal.

2. Add the ROS Noetic repository to your sources list by running the following command:
    ```
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-noetic.list'
    ```

3. Set up your keys by running the following command:
    ```
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    ```

4. Update your package list by running the following command:
    ```
    sudo apt update
    ```

5. Install ROS Bridge by running the following command:
    ```
    sudo apt install ros-noetic-rosbridge-server
    ```

6. Verify the installation by running the following command:
    ```
    roslaunch rosbridge_server rosbridge_websocket.launch
    ```

    If the installation was successful, you should see output indicating that the ROS Bridge server is running.

Congratulations! You have successfully installed ROS Bridge in ROS Noetic on Ubuntu 20.

# Running Mock Web Interface

To run the mock web interface, follow the steps below:

1. Launch the web interface using the `roslaunch` command:

```bash
roslaunch launch/mock_web_interface.launch
```

1.　Start the web server by executing the start script:

```bash
./scripts/start_web_server.sh
```

# Troubleshooting Guide

If you notice that the changes to your source code are not being reflected when you load the web page, it could be due to browser caching. Follow the steps below to ensure that your latest changes are loaded:

## Steps to Clear Cache and Force Reload

### Google Chrome:
1. Open the Chrome DevTools. You can do this by pressing `F12` or right-clicking anywhere on the page and selecting `Inspect`.
2. Right-click the refresh button and select `Empty Cache and Hard Reload`.

### Firefox:
1. Open the Firefox DevTools. You can do this by pressing `F12` or right-clicking anywhere on the page and selecting `Inspect`.
2. Click the `Network` tab, and check the `Disable cache` option.
3. Reload the page.