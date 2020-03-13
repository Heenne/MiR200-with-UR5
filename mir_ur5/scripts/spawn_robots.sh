#!/bin/bash

terminator -e "roslaunch mir_ur5 spawn_robot0.launch"&
sleep 1
terminator -e "roslaunch mir_ur5 spawn_robot1.launch"&

echo "Finished spawning robots."