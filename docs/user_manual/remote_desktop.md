# Setting Up TigerVNC Server on a Remote Ubuntu Machine

This guide will walk you through the process of setting up a TigerVNC server on a remote Ubuntu machine without a physical monitor. This is achieved by configuring a virtual display.

## Step 1: Install TigerVNC Server

First, we need to install the TigerVNC server on the Ubuntu machine. Follow these steps:

1. **Update the package list:**
    Use the `apt-get update` command to update the package list.
    ```bash
    sudo apt-get update
    ```

2. **Install the TigerVNC server package:**
    Use the `apt-get install` command to install the package.
    ```bash
    sudo apt-get install tigervnc-standalone-server tigervnc-common
    ```

## Step 2: Install XFCE Desktop Environment

We need to install the XFCE desktop environment, which will be used for the remote desktop.

1. **Install XFCE:**
    ```bash
    sudo apt-get install xfce4 xfce4-goodies
    ```

## Step 3: Configure a Virtual Display

It is possible to create a dummy display in Ubuntu 20.04 to use TigerVNC for remote desktop access even without a physical monitor connected. Here are the steps:

1. **Install the required packages:**
    ```bash
    sudo apt-get install xserver-xorg-video-dummy
    ```

2. **Create a new X server configuration file:**
    ```bash
    sudo nano /etc/X11/xorg.conf
    ```

3. **Add the following lines to the file:**
    ```
    Section "Device"
        Identifier "Configured Video Device"
        Driver "dummy"
    EndSection

    Section "Monitor"
        Identifier "Configured Monitor"
    EndSection

    Section "Screen"
        Identifier "Configured Screen"
        Monitor "Configured Monitor"
        Device "Configured Video Device"
        SubSection "Display"
            Modes "1920x1080"
        EndSubSection
    EndSection
    ```
    You can change the resolution in the `Modes` line as per your requirement.

4. **Save the file and exit.**

5. **Restart the X server:**
    ```bash
    sudo systemctl restart gdm
    ```

## Step 4: Create and Enable the VNC Server Service

We need to create a systemd service to start TigerVNC at boot.

1. **Set the VNC password:**
    ```bash
    vncpasswd
    ```

2. **Create the service file:**
    ```bash
    sudo nano /etc/systemd/system/vncserver@.service
    ```

3. **Add the following configuration to the service file:**
    ```
    [Unit]
    Description=Start TigerVNC server at startup
    After=syslog.target network.target

    [Service]
    Type=forking
    User=mobi
    PAMName=login
    PIDFile=/home/mobi/.vnc/%H%i.pid
    ExecStartPre=-/usr/bin/vncserver -kill :%i > /dev/null 2>&1
    ExecStart=/usr/bin/vncserver :%i -geometry 1920x1080 -depth 24 -localhost no
    ExecStop=/usr/bin/vncserver -kill :%i

    [Install]
    WantedBy=multi-user.target
    ```

    Replace `mobi` with your username.

4. **Reload the systemd daemon:**
    ```bash
    sudo systemctl daemon-reload
    ```

5. **Enable the VNC server service to start on boot:**
    ```bash
    sudo systemctl enable vncserver@1.service
    ```

6. **Start the VNC server service:**
    ```bash
    sudo systemctl start vncserver@1.service
    ```

    **Note:** Even if you encounter an error while running the command `sudo systemctl status vncserver@1.service`, you can ignore it. Try to reboot and remote it using the IP address and display number, for example, `192.168.2.2:1`.

## Step 5: Configure the VNC Startup Script

1. **Create or edit the xstartup file:**
    ```bash
    nano ~/.vnc/xstartup
    ```

2. **Add the following lines to the file:**
    ```bash
    #!/bin/bash
    xrdb $HOME/.Xresources
    startxfce4 &
    xfce4-terminal &
    ```

3. **Make the xstartup file executable:**
    ```bash
    chmod +x ~/.vnc/xstartup
    ```

## Step 6: Connect to the VNC Server

Finally, we can connect to the VNC server:

1. **Retrieve the IP address of the Ubuntu machine:**
    Use the `ifconfig` command to find the IP address of the machine.
    ```bash
    ifconfig
    ```

2. **Use a VNC viewer (e.g., TigerVNC Viewer) to connect to the IP address on port `5901` (e.g., `192.168.1.2:5901`).**

    **Note:** Use RealVNC Viewer for remoting.

3. **While in a remote desktop, use xfce4-terminal for terminal by clicking run program and typing `terminal`.**

By following these steps, you should be able to set up the TigerVNC server on a remote Ubuntu machine without needing a physical monitor. The virtual display configuration ensures that the desktop environment is accessible remotely.
