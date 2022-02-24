# Logistics-Delivery-Drone

Hexacopter program developed for LAPAN drone delivery project

## Prerequisites

1. ROS (Robot Operating System) Kinetic kame
2. OpenCV library
3. C++ 11

### ROS packages

4. MavROS - used for mavlink communication with Pixhawk
5. video_stream_opencv
6. aruco_ros

### Optional (Needed for simulation)

1. Gazebo 7 or Gazebo 9
2. Ardupilot SITL (Software in the loop)
3. Mission planner - running on ubuntu 16.04 through wine

## Running the simulation

```
roscore
rosrun gazebo_ros gazebo --verbose worlds/iris_arducopter_runway.world
sim_vehicle.py -f gazebo-iris --console
roslaunch drone mission1_sim.launch
```

## Connecting to real vehicle

```
roscore
roslaunch drone mission1.launch
```

## Notes on Gazebo simulation

1. Consider using [Swiftgust's version](https://github.com/SwiftGust/ardupilot_gazebo) of ardupilot-gazebo simulation for ubuntu 16.04
2. Confirm that model and world paths are exported to make sure Gazebo is able to find the corresponding files.
```
source /usr/share/gazebo/setup.sh

export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=~/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}
```

3. [gazebo-ros camera plugin](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/jade-devel/gazebo_plugins/src/gazebo_ros_camera.cpp) is needed to be compiled and added to gimbal's .udf file to publish simulation camera to rostopic. Clone and catkin build it.

the gimbal .udf file can be found at:

```
~/ardupilot_gazebo/models_gazebo/gimbal_small_2d
```

note that the ardupilot_gazebo is Swiftgust's

Plugin has to be added inside `<sensor> </sensor>` [example](http://gazebosim.org/tutorials?tut=ros_gzplugins):

```
  <!-- camera -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <!-- Noise is sampled independently per pixel on each frame.
               That pixel's noise value is added to each of its color
               channels, which at that point lie in the range [0,1]. -->
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>rrbot/camera1</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>
```

4. Gazebo must be run with:
`rosrun gazebo_ros gazebo`
instead of
`gazebo`
otherwise, this error would appear:

```
A ROS node for Gazebo has not been initialized, unable to load plugin.
Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package
```

5. CMake needs to be updated to the latest version to prevent errors when compiling ardupilot gazebo plugin.
6. Please be advised to use only one version of gazebo. In case two versions have been previously installed, make sure to uninstall one of the versions completely as it might cause misplacement of installation files.
