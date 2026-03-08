#!/bin/bash
# List Gazebo Transport topics that look like IMU or point cloud (run while sim is running).
echo "Gazebo topics containing 'imu' or 'points' (run with sim + excavator spawned):"
gz topic -l 2>/dev/null | grep -E 'imu|points' || echo "No matches. Ensure Gazebo is running and excavator is spawned; try pressing Play."
