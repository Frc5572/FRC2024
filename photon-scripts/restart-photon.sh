#!/bin/bash

while :
do
    error=`systemctl status photonvision.service | grep -i "Input was empty!"`
    if [[ $? -eq 0 ]];
    then
        dmesg | grep uvc > "/opt/uvc-$(date +%y%m%d%H%M%S).txt"
        systemctl stop photonvision.service
        modprobe uvcvideo
        systemctl start photonvision.service
        sleep 15
    fi
    sleep 1
done
