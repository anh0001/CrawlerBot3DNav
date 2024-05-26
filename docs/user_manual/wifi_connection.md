# Maintaining Persistent WiFi Connection in Ubuntu 20.04

If you're experiencing intermittent WiFi disconnections, one potential solution is to disable WiFi power management. Here's how to do it:

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