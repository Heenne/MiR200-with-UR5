#!/bin/bash

number=$1 #First argument handed into the script will be the number of ur5 controller slaves spawned

for((counter = 0; counter <$number; ++counter))
do
    echo "Spawning robot number: $counter"
    terminator -e "roslaunch ur5_controller ur5_controller.launch index:=$counter"&
    sleep 1
done
echo "Finished spawning ur5 controller"