
# Ethernet Configuration Setup using Netplan

This guide provides steps to configure Ethernet interfaces for a fixed IP and DHCP on a Linux system using Netplan. Specifically, Ethernet interface `enp1s0` will be set to a fixed IP of 192.168.1.50 for a Livox Mid-360 LiDAR sensor, and Ethernet interface `enp3s0` will be set to DHCP for internet connectivity.

## Steps

### 1. Open Netplan Configuration File

Netplan configuration files are typically located in the `/etc/netplan` directory and have a `.yaml` extension. Open the existing configuration file or create a new one if it does not exist.

```sh
sudo nano /etc/netplan/01-netcfg.yaml
```

### 2. Configure Ethernet Interfaces

Add the following configuration to set Ethernet interface `enp1s0` to a fixed IP and `enp3s0` to use DHCP.

```yaml
network:
  version: 2
  ethernets:
    enp1s0:
      dhcp4: no
      addresses:
        - 192.168.1.50/24
      gateway4: 192.168.1.1
      nameservers:
        addresses:
          - 8.8.8.8
          - 8.8.4.4
    enp3s0:
      dhcp4: yes
```

### 3. Apply the Configuration

Apply the Netplan configuration with the following command:

```sh
sudo netplan apply
```

### 4. Verify Configuration

Verify the configuration of the network interfaces to ensure they are set correctly.

```sh
ip a
```

Ensure `enp1s0` has the IP address `192.168.1.50` and `enp3s0` is assigned an IP address by the DHCP server.