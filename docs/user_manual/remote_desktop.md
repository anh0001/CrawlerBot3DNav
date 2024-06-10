
# Setting Up RealVNC Server on a Remote Ubuntu Machine

This guide will walk you through the process of setting up a RealVNC server on a remote Ubuntu machine without a physical monitor. This is achieved by configuring a virtual display.

## Step 1: Install RealVNC Server

First, we need to install the RealVNC server on the Ubuntu machine. Follow these steps:

1. **Download the RealVNC server package:**
    Use the `wget` command to download the package.
    ```bash
    wget https://www.realvnc.com/download/file/vnc.files/VNC-Server-6.7.2-Linux-x64.deb
    ```
    Note: Replace the URL with the latest version if necessary.

2. **Install the downloaded package:**
    Use the `dpkg` command to install the package.
    ```bash
    sudo dpkg -i VNC-Server-6.7.2-Linux-x64.deb
    ```

3. **Fix any dependency issues:**
    If there are any dependency issues, use the `apt-get install -f` command to fix them.
    ```bash
    sudo apt-get install -f
    ```

## Step 2: Configure a Virtual Display

It is possible to create a dummy display in Ubuntu 20.04 to use AnyDesk for remote desktop access even without a physical monitor connected. Here are the steps:

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
            Modes "1920x1024"
        EndSubSection
    EndSection
    ```
    You can change the resolution in the `Modes` line as per your requirement.

4. **Save the file and exit.**

5. **Restart the X server:**
    ```bash
    sudo systemctl restart gdm
    ```

6. After restarting, you should be able to start AnyDesk and connect to the dummy display.

## Step 3: Start the VNC Server or AnyDesk

Now, we can start the VNC server with the virtual display:

1. **Start the VNC server with the virtual display:**
    Use the `systemctl start` command to start the server.
    ```bash
    sudo systemctl start vncserver-x11-serviced.service
    ```

2. **Enable the VNC server to start on boot:**
    Use the `systemctl enable` command to ensure the server starts on boot.
    ```bash
    sudo systemctl enable vncserver-x11-serviced.service
    ```

## Step 4: Connect to the VNC Server

Finally, we can connect to the VNC server:

1. **Retrieve the IP address of the Ubuntu machine:**
    Use the `ifconfig` command to find the IP address of the machine.
    ```bash
    ifconfig
    ```

2. **Use a VNC viewer (e.g., RealVNC Viewer) to connect to the IP address.**

By following these steps, you should be able to set up the RealVNC server on a remote Ubuntu machine without needing a physical monitor. The virtual display configuration ensures that the desktop environment is accessible remotely.
