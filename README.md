# Logistics-Delivery-Drone
Hexacopter program developed for LAPAN drone delivery project

## Prerequisites
1. ROS (Robot Operating System) Kinetic kame
2. OpenCV library
3. C++ 11
### ROS package
4. MavROS - used for mavlink communication with Pixhawk
5. video_stream_opencv 
6. aruco_ros

### Optional (Needed for simulation)
1. Gazebo 7 or Gazebo 9
2. Ardupilot SITL (Software in the loop)
3. Mission planner - running on ubuntu 16.04 through wine

## Running the simulation
```bash 
roscore
rosrun gazebo_ros gazebo --verbose worlds/iris_arducopter_runway.world
sim_vehicle.py -f gazebo-iris --console
roslaunch drone mission1_sim.launch
```

## Connecting to real vehicle
```bash 
roscore
roslaunch drone mission1.launch
```

## Notes on Gazebo simulation
1. Confirm that model and world paths are exported to make sure Gazebo is able to find the corresponding files.
2. CMake needs to be updated to the latest version to prevent error when compiling ardupilot gazebo plugin.
3. Please be advised to use only one version of gazebo. In case that two versions have been previously installed, make sure to uninstall one of the version completely as it might causing misplacement of installation files.
4. gazebo-ros camera plugin is needed to to be compiled and added to gimbal's URDF file to publish simulation camera to rostopic.
