# Wi-Fi Provisioning Dashboard

This starts the dashboard after reboot. It accepts Wi-Fi credentials, writes
the `wlan0` wpa_supplicant configuration, then restarts
`wpa_supplicant@wlan0` and requests DHCP. The configuration is root-readable
only, but contains the submitted passphrase; protect access to the robot.
Saving may briefly drop the browser connection. Use it only on a trusted local
network because the browser submits credentials over HTTP.

```bash
sudo install -m 0644 jetson_wifi/systemd/wifi-dashboard-preview.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now wifi-dashboard-preview.service
```

Check it with:

```bash
systemctl status wifi-dashboard-preview.service
```

The dashboard is available at `http://<robot-ip>:8095/`.
