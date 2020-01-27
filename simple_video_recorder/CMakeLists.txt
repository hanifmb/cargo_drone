cmake_minimum_required(VERSION 2.8)
project(simple_video_recorder)

find_package(catkin REQUIRED COMPONENTS camera_calibration_parsers cv_bridge dynamic_reconfigure image_transport message_filters message_generation rosconsole roscpp std_srvs stereo_msgs)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  record_trigger.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
)

add_executable(simple_video_recorder src/simple_video_recorder.cpp)
target_link_libraries(simple_video_recorder ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
)

install(TARGETS simple_video_recorder
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



