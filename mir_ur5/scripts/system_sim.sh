#!/bin/bash

#Start the simulation environment
terminator -e "roslaunch mir_ur5 system_sim.launch"&

sleep 3

#Spawn the robot models into the simulation environment
terminator -e "roslaunch control_ur5_test spawn_separated_block.launch"&
./spawn_robots.sh

sleep 10

#Start the ur5 controller slaves
./launch_ur5_controller.sh 2&

# sleep 10
# terminator -e "roslaunch ur5_controller ur5_controller_manager.launch"&

echo "Finished setting up sim environment."
$SHELL