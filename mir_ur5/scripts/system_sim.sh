#!/bin/bash

#Start the simulation environment
terminator -e "roslaunch mir_ur5 system_sim.launch"&

sleep 3

#Spawn the robot models into the simulation environment
terminator -e "roslaunch control_ur5_test spawn_separated_block.launch"&
./spawn_robots.sh

sleep 5

#Start the ur5 controller slaves
./launch_ur5_controller_slaves.sh 2

sleep 5

terminator -e "roslaunch ur5_controller robot_ur5_master.launch"&

echo "Finished setting up sim environment."
$SHELL