# Install WiFi Driver

1. **Turn off Secure Boot in BIOS:** Ensure that Secure Boot is disabled in your BIOS settings.

2. **Clone the WiFi Driver Repository:** Open the terminal and clone the driver repository from GitHub:

    ```bash
    git clone https://github.com/HRex39/rtl8852be.git
    ```

3. **Navigate to the Cloned Repository:** Change to the directory of the cloned repository:

    ```bash
    cd rtl8852be
    ```

4. **Compile and Install the Driver:** Run the following commands to compile and install the driver:

    ```bash
    make -j8
    sudo make install
    sudo modprobe 8852be
    ```

By following these steps, you will install the WiFi driver, which may help in maintaining a persistent WiFi connection.

# Maintaining Persistent WiFi Connection in Ubuntu 20.04

If you're experiencing intermittent WiFi disconnections, another potential solution is to disable WiFi power management. Here's how to do it:

## Disable Power Management for WiFi

1. **Open the Terminal:** You can do this by pressing `Ctrl + Alt + T`.

2. **Edit the Network Manager Configuration File:** Enter the following command to open the configuration file in a text editor:

    ```bash
    sudo nano /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf
    ```

3. **Modify the Configuration File:** Add the following line to the file:

    ```bash
    wifi.powersave = 2
    ```

    Then save and close the file.

4. **Restart the Network Manager Service:** Finally, restart the network manager service by entering the following command:

    ```bash
    sudo service network-manager restart
    ```

By following these steps, you disable WiFi power saving modes, which could potentially resolve the issue of intermittent disconnections.

> **Note:** This solution is still under review for checking the validity.
