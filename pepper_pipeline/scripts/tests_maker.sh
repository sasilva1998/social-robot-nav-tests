#!/bin/bash
for i in {0..100}
do
    echo "========================================================\n"
    echo "This is the $i th run\n"
    echo "========================================================\n"
    source /home/sasm/.irohms/setup_irohms.bash
    python /home/sasm/people_sim_ws/src/pedsim_ros/pedsim_simulator/scripts/metrics_recorder_launcher.py
    killall -9 gzserver gzclient
done