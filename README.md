# Logistics-Delivery-Drone
Hexacopter program and simulation developed for LAPAN drone delivery project

## Prerequisites
1. ROS (Robot Operating System) Kinetic kame
2. OpenCV library
3. MavRos - used for mavlink communication with Pixhawk
4. C++ 11

Optional (Needed for simulation)
1. Gazebo 7 or Gazebo 9
2. Ardupilot SITL (Software in the loop)
3. Mission planner - running on ubuntu 16.04 through wine

## Notes on Gazebo simulation
1. Make sure the model and world paths are exported to make sure Gazebo able to find the corresponding files.

```bash
#example
Some code 
```

2. Having two versions of gazebo might messed up the ardupilot-gazebo installation process. Make sure to fully uninstall the unused version of gazebo.
