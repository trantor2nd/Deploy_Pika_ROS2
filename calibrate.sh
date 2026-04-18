#!/bin/bash
# libsurvive VR 追踪设备标定
export LD_LIBRARY_PATH=/home/data/Project/pika_ros/install/libsurvive/lib:$LD_LIBRARY_PATH
cd /home/data/Project/pika_ros/install/libsurvive/bin && ./survive-cli --force-calibrate
