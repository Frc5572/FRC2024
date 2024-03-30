#!/bin/bash

while :
do
	$error = `systemctl status photonvision.service | grep -i "input was empty"`
    if [[ $? -eq 0 ]];
    then
        modprobe uvcvideo
        systemctl restart photonvision.service
    fi
	sleep 1
done
