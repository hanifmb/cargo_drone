cmake_minimum_required(VERSION 2.8.3)
project(cargo_drone)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(OpenCV REQUIRED)
#find_package(gazebo REQUIRED)
#find_package(catkin REQUIRED dynamic_reconfigure)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  roscpp
  std_msgs
  image_transport
  message_generation
  dynamic_reconfigure
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


add_service_files(
  FILES
  cmd.srv
)

generate_dynamic_reconfigure_options(
  cfg/drone_param.cfg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
#include_directories(${GAZEBO_INCLUDE_DIRS})

#link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(controller src/controller.cpp)
add_dependencies(controller ${PROJECT_NAME}_gencfg)

target_link_libraries(controller ${catkin_LIBRARIES})



