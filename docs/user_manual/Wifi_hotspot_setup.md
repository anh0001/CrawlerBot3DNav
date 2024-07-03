
# Setting Up a WiFi Hotspot on Ubuntu 20 with Auto-Start

This guide details the steps to set up a WiFi hotspot on Ubuntu 20 using `nmcli`, ensuring it automatically starts on reboot.

## Initial Setup

1. **Set up the hotspot using `nmcli`:**

   ```bash
   nmcli dev wifi hotspot ifname wlp2s0 ssid mobi_unibo password "mobimobi"
   ```

2. **Identify your hotspot connection name:**

   ```bash
   nmcli connection show
   ```

   Look for the name of the hotspot connection you created. It should be `Hotspot`.

3. **Configure the hotspot connection to auto-connect:**

   ```bash
   nmcli connection modify "Hotspot" connection.autoconnect yes
   ```

4. **Configure the Hotspot** - This step involves setting up the hotspot with a specific IP address range and defining the gateway. This is crucial for managing the network traffic through the hotspot.

   ```bash
   #!/bin/bash
   nmcli connection modify "Hotspot" ipv4.addresses 192.168.77.1/24
   nmcli connection modify "Hotspot" ipv4.gateway 192.168.77.1
   nmcli connection modify "Hotspot" ipv4.method shared
   nmcli connection up "Hotspot"
   ```

## Ensure Hotspot Starts on Reboot (Optional)

4. **Create a script to ensure the hotspot starts on reboot:**

   Create a new script file, for example, `start_hotspot.sh`:

   ```bash
   sudo nano /usr/local/bin/start_hotspot.sh
   ```

   Add the following content to the script:

   ```bash
   #!/bin/bash
   nmcli connection up "Hotspot"
   ```

5. **Make the script executable:**

   ```bash
   sudo chmod +x /usr/local/bin/start_hotspot.sh
   ```

6. **Create a systemd service to run the script at startup:**

   Create a new service file:

   ```bash
   sudo nano /etc/systemd/system/start_hotspot.service
   ```

   Add the following content to the service file:

   ```ini
   [Unit]
   Description=Start WiFi Hotspot

   [Service]
   ExecStart=/usr/local/bin/start_hotspot.sh

   [Install]
   WantedBy=multi-user.target
   ```

7. **Enable the service to start on boot:**

   ```bash
   sudo systemctl enable start_hotspot.service
   ```

8. **Start the service to test if it works:**

   ```bash
   sudo systemctl start start_hotspot.service
   ```

## Troubleshooting

If you encounter any issues, you can check the status of the service for troubleshooting:

```bash
sudo systemctl status start_hotspot.service
```

This configuration ensures that your hotspot with the SSID `mobi_unibo` and password `mobimobi` using the interface `wlp2s0` will automatically start on reboot, with the connection name `Hotspot`.
