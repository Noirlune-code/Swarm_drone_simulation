#!/bin/bash

# Base command directory
cd ~/Softwares/ardupilot || { echo "Directory not found"; exit 1; }

# Function to launch SITL in a new GNOME Terminal
launch_instance() {
    local instance_id=$1
    gnome-terminal -- bash -c "./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I$instance_id; exec bash"
}

# Launch 5 instances
for i in {0..4}; do
    launch_instance $i
    sleep 1  # Small delay to avoid race conditions
done
