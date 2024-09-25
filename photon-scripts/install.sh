#!/bin/bash
nano /opt/photonvision/restart-photon.sh
chmod +x /opt/photonvision/restart-photon.sh
nano /etc/systemd/system/photon-camera-fix.service
systemctl daemon-reload
systemctl restart photon-camera-fix
systemctl status photon-camera-fix
