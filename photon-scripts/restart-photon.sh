#!/bin/bash

while :
do
    error=`systemctl status photonvision.service | grep -i "Input was empty!"`
    if [[ $? -eq 0 ]];
    then
        echo "Photon Vision reporting 'Input was empty'"
        dmesg | grep uvc > "/opt/uvc-$(date +%y%m%d%H%M%S).txt"
        echo "Stopping Photon Vision"
        systemctl stop photonvision.service
        sleep 1
        modprobe uvcvideo
        sleep 1
        echo "Starting Photon Vision"
        systemctl start photonvision.service
        sleep 15
    fi
    sleep 1
done
