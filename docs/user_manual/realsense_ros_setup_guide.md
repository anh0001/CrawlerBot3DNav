# Step-by-Step Guide

## 1. Install Intel RealSense SDK
First, you need to install the Intel RealSense SDK, which provides the necessary libraries and tools to work with RealSense cameras.

Add the Intel RealSense repository and key:
```bash
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u
```

Update the package list and install the libraries:
```bash
sudo apt update
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

Verify the installation by running the RealSense viewer:
USB port issues: Ensure that your RealSense camera is properly connected to a USB 3.0 port and that the port itself is functioning correctly. Try different USB ports or computers to isolate hardware issues.
```bash
realsense-viewer
```

## 2. Install ROS Noetic
If ROS Noetic is not already installed, follow these steps:

Set up the sources list:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Add the ROS key:
```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Update the package list and install ROS Noetic:
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```

Initialize rosdep:
```bash
sudo rosdep init
rosdep update
```

Set up the ROS environment:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. Install the RealSense ROS Wrapper
The RealSense ROS wrapper allows you to use RealSense cameras with ROS.

Install the ROS wrapper for RealSense:
```bash
sudo apt install ros-noetic-realsense2-camera ros-noetic-ddynamic-reconfigure ros-noetic-realsense2-camera ros-noetic-realsense2-description
```

Clone the RealSense ROS wrapper repository, but use ros1-legacy as we are going to use ros1:
```bash
cd src/
git subtree add --prefix realsense-ros https://github.com/IntelRealSense/realsense-ros.git ros1-legacy --squash
cd ..
catkin_make
```

Source the workspace:
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 4. Launch the RealSense Node
Finally, you can launch the RealSense node to start using the D455 camera with ROS.

Launch the RealSense node:
```bash
roslaunch realsense2_camera rs_camera.launch
```

This setup will allow you to use the Intel RealSense D455 camera with ROS Noetic on Ubuntu 20.04. The RealSense ROS wrapper provides various topics and services to interact with the camera, such as depth images, color images, and IMU data.

# Important Note

Ensure that the RealSense D455 camera's USB cable is connected to a high-speed USB port on the Intel NUC box.