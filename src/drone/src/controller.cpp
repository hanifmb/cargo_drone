#include <ros/ros.h>
#include "std_msgs/String.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include "drone/cmd.h"



#include <boost/thread.hpp>
#include <sstream>

mavros_msgs::State current_state;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient takeoff_client;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void spinThread(){
    ros::spin();
}

bool decoder(drone::cmd::Request  &req,
    drone::cmd::Response &res){
    
    res.success = true;
    return true;
}

void move_front(){
    
}

void move_back(){
    
}

void move_right(){
    
}

void move_left(){
    
}

void mission1_start(){
 //Set mode to GUIDED
  mavros_msgs::SetMode set_mode;
  set_mode.request.custom_mode = "GUIDED";

  while(current_state.mode != "GUIDED"){  
        ROS_INFO("Switching to GUIDED");
        if(set_mode_client.call(set_mode) && set_mode.response.mode_sent){
            ROS_INFO("GUIDED enabled");
            break;
        }
        ros::Duration(5).sleep();
    }

  //Arming
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  
  while(!current_state.armed){
        if(arming_client.call(arm_cmd) && arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
            break;
        }
        ros::Duration(5).sleep();
    }
  
  //Take off
  mavros_msgs::CommandTOL takeoff_param;
  takeoff_param.request.altitude = 10.0;

  if(takeoff_client.call(takeoff_param)){
      ROS_INFO("Taking off");
  }
  
  //Switch to AUTO
  set_mode.request.custom_mode = "AUTO";
  while(current_state.mode != "AUTO"){
      if(set_mode_client.call(set_mode) && set_mode.response.mode_sent){
          ROS_INFO("AUTO enabled");
      }
      ros::Duration(1).sleep();
  }  
}

int main(int argc, char **argv)
{

  // ROS node initialization
  ros::init(argc, argv, "drone_control");
  ros::NodeHandle n;

  //Thread initialization
  boost::thread spin_thread(&spinThread);

  //Subscriber
  ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

  //Service
  arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

  ros::ServiceServer service = n.advertiseService("drone_cmd", decoder);
  
  ROS_INFO("ROSnode initialized");

  ros::Rate rate(20);
  spin_thread.join();
  return 0;
 }