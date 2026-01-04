#!/usr/bin/env bash

set -e

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <ros_image_topic> <bitrate>"
    exit 1
fi

ROS_WS="/home/radxa-x4/Documents/ros-gst-bridge"

cd "$ROS_WS"
source "$ROS_WS/install/setup.bash"

exec gst-launch-1.0 \
    rosimagesrc ros-topic="$1" \
    ! videoconvert \
    ! queue \
    ! video/x-raw \
    ! x264enc tune=zerolatency bitrate="$2" speed-preset=superfast \
    ! rtph264pay config-interval=1 pt=96 \
    ! udpsink host=192.168.11.4 port=5000
