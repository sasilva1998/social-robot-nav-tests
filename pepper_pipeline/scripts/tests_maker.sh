#!/bin/bash
for i in $(seq 1 100)
do
    echo "========================================================\n"
    echo "This is the $i th run\n"
    echo "========================================================\n"
    python /home/sasm/ros/melodic/system/src/pepper_pipeline/scripts/tests_launcher.py
    killall -9 gzserver gzclient
done