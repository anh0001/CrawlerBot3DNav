
# Lidar Livox Mid-360 Subtree Creation Guide

This guide will walk you through the process of creating subtrees for Lidar Livox Mid-360.

# Create a Subtree of Livox SDK2

First, we need to create a subtree of the Livox SDK2. Run the following command:

```bash
git subtree add --prefix=src/Livox-SDK2 https://github.com/Livox-SDK/Livox-SDK2.git master --squash
```

After running the command, follow the installation instructions in the readme file.

# Create subtree of github livox_ros_driver2

To create a subtree of Lidar Livox Mid-360, use the following command:

```bash
git subtree add --prefix=src/livox_ros_driver2 https://github.com/Livox-SDK/livox_ros_driver2.git master --squash
```

Run this command
```bash
src/livox_ros_driver2/build.sh ROS1
```

# Configuring Ubuntu to Receive Internet via Wi-Fi and Set Static IP on LAN Interface

According to the provided sources, all Livox Mid-360 LiDAR sensors are set to static IP address mode by default with an IP address of 192.168.1.1XX, where XX stands for the last two digits of the Livox Mid-360 LiDAR sensor's serial number.

To get the IP address of your specific Livox Mid-360 LiDAR sensor, follow these steps:
1. Check the serial number printed on your Livox Mid-360 device.
2. The default IP address will be 192.168.1.1XX, where XX are the last two digits of the serial number.

For example, if your Livox Mid-360's serial number is 123456, the default IP address would be 192.168.1.156.
The default subnet mask for all Livox Mid-360 LiDAR sensors is 255.255.255.0, and the default gateway is 192.168.1.1.

## Step 1: Connect to Wi-Fi
Ensure your Wi-Fi connection is set up and connected to the internet. You can manage Wi-Fi connections using Network Manager or command-line tools like `nmcli`.

## Step 2: Configure Static IP on `enp3s0`
Set a static IP address for your LAN interface (`enp3s0`). You can do this by editing the Netplan configuration files.

### Steps to configure static IP using Netplan:

1. **Open the Netplan configuration file**:
   The configuration files are usually located in `/etc/netplan/`. The file might be named something like `01-netcfg.yaml` or similar.

   ```bash
   sudo nano /etc/netplan/01-netcfg.yaml
   ```

2. **Edit the configuration file**:
   Add or modify the configuration to set a static IP for the `enp3s0` interface. Here is an example configuration:

   ```yaml
   network:
     version: 2
     renderer: networkd
     ethernets:
       enp3s0:
         dhcp4: no
         addresses:
           - 192.168.1.50/24
         gateway4: 192.168.1.1
         nameservers:
           addresses:
             - 8.8.8.8
             - 8.8.4.4
   ```

   Adjust the `addresses`, `gateway4`, and `nameservers` fields according to your network settings. If you don't need a gateway or DNS for the sensor network, you can omit those lines.

3. **Apply the configuration**:
   Save the file and apply the changes using Netplan.

   ```bash
   sudo netplan apply
   ```

## Step 3: Verify the Configuration
After applying the configuration, verify that both the Wi-Fi connection and the static IP on `enp3s0` are correctly set up.

- **Check Wi-Fi connection**:

  ```bash
  nmcli device status
  ```

- **Check LAN interface**:

  ```bash
  ip a show enp3s0
  ```

## Step 4: Connecting to the Sensor
Ensure that the sensor is correctly connected to the LAN interface (`enp3s0`) and configured to communicate within the same network range. You can test the connection by pinging the sensor's IP address:

```bash
ping <sensor-ip-address>
```

# Configure MID360_config.json
{
  "lidar_summary_info" : {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info" : {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.50",
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.50",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.50",
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.50",
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.192",
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}

# Notes
## To delete a subtree that you have added to your Git repository, you need to follow a few steps. Here’s how you can do it:

1. Remove the subtree directory: First, delete the subtree directory from your working directory. In your case, it would be `src/livox_ros_driver`.

2. Commit the deletion: Commit the changes to remove the directory from the repository.
   ```bash
   git rm -r src/livox_ros_driver2
   git commit -m "Remove livox_ros_driver2 subtree"
   ```

3. Remove the subtree reference from Git history: Use the `filter-branch` command to remove all references to the subtree from your Git history. This is a bit more advanced and should be used with caution as it rewrites the history of your repository.
   ```bash
   git filter-branch -f --tree-filter 'rm -rf src/livox_ros_driver2' HEAD
   ```

4. Force push the changes (optional): If you are working on a shared repository, you might need to force push the changes to the remote repository.
   ```bash
   git push origin main --force
   ```

### Note:
Rewriting history can cause problems if others are working on the same repository. Make sure to inform your team and coordinate accordingly.
