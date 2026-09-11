#!/bin/bash 


sudo v4l2-ctl --set-ctrl=rotate=180
sudo v4l2rtspserver -F 5 -H 600 -W 800 /dev/video0
