#include <ros/ros.h>
#include "std_msgs/String.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include "cargo_drone/cmd.h"

#include <boost/thread.hpp>
#include <sstream>

class DroneController{
    private:
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient takeoff_client;
    ros::ServiceServer service;
    ros::Subscriber state_sub;
    ros::Subscriber relative_altitude_sub;
    ros::Subscriber pose1_sub;
    ros::Publisher vel_setpoint_pub;
    std_msgs::Float64 cur_rel_altitude;
    geometry_msgs::Pose cur_pose_1;
    geometry_msgs::TwistStamped vel_setpoint;
    mavros_msgs::State current_state;

    double last_pose1_time;
    bool go_down;
    
    boost::thread mission_thread;

    double x_kp,  y_kp, z_kp;
    double ar_timeout, diameter_thresh, lower_target_alt, 
            upper_target_alt, takeoff_alt;
    
    public:
    DroneController(ros::NodeHandle* nh)
    {
        //Subscriber
        state_sub = nh->subscribe<mavros_msgs::State>("mavros/state", 10, &DroneController::state_cb, this);
        relative_altitude_sub = nh->subscribe<std_msgs::Float64>("/mavros/global_position/rel_alt", 10, &DroneController::relative_altitude_cb, this);
        pose1_sub = nh->subscribe<geometry_msgs::Pose>("/aruco_simple/pose", 10, &DroneController::pose1_cb, this);

        vel_setpoint_pub = nh->advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1000);

        //Service
        service = nh->advertiseService("/controller/drone_cmd", &DroneController::decoder_cb, this);

        arming_client = nh->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client = nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        takeoff_client = nh->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

        go_down = false;

        //param
        nh->getParam("cargo_drone/x_control/kp", z_kp);
        nh->getParam("cargo_drone/y_control/kp", y_kp);
        nh->getParam("cargo_drone/z_control/kp", z_kp);
        nh->getParam("cargo_drone/ar_detect/timeout", ar_timeout);
        nh->getParam("cargo_drone/ar_detect/go_down/diameter_thresh", diameter_thresh);
        nh->getParam("cargo_drone/go_down/lower_alt", lower_target_alt);
        nh->getParam("cargo_drone/go_up/upper_alt", upper_target_alt);
        nh->getParam("cargo_drone/take_off/alt", takeoff_alt);

    }

    double proporsional(double Kp, double setpoint, double state){
        return Kp*(state-setpoint);
    }

    float centering_start(){

        double current_time = ros::Time::now().toSec();
        //double duration = 5;
        
        ros::Rate send_vel_rate(30);
        
        while (current_time - last_pose1_time < ar_timeout){
            ROS_ERROR("ini di print selama 5 detik, last pose: %f", current_time - last_pose1_time);

            double vel_x = proporsional(x_kp, 0, cur_pose_1.position.x);
            double vel_y = proporsional(y_kp, 0, cur_pose_1.position.y);
            double vel_z = proporsional(z_kp, lower_target_alt, cur_rel_altitude.data);       

            vel_setpoint.twist.linear.x = vel_x;
            vel_setpoint.twist.linear.y = -vel_y;

            if(vel_setpoint.twist.linear.x < diameter_thresh &&
            vel_setpoint.twist.linear.x > -diameter_thresh && 
            vel_setpoint.twist.linear.y < diameter_thresh && 
            vel_setpoint.twist.linear.y > -diameter_thresh &&
            !go_down)
            {
                go_down = true;
            }

            if(go_down){
                vel_setpoint.twist.linear.z = -vel_z;
            }
            else{
                vel_setpoint.twist.linear.z = 0.0;
            }

            vel_setpoint_pub.publish(vel_setpoint);

            current_time = ros::Time::now().toSec();
            send_vel_rate.sleep();
        } 
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
        
        //Add duration between arming and take off
        ros::Duration(2).sleep();

        //Take off
        mavros_msgs::CommandTOL takeoff_param;
        takeoff_param.request.altitude = takeoff_alt;

        if(takeoff_client.call(takeoff_param)){
            ROS_INFO("Taking off");
        }

        //wait until altitude is reached
        while(1){
            std::cout << "altitude" << cur_rel_altitude.data << std::endl; 
            ros::Duration(0.5).sleep();
        }

        ROS_INFO("Altitude is reached, switching to AUTO");
        
        //Switch to AUTO
        set_mode.request.custom_mode = "AUTO";
        while(current_state.mode != "AUTO"){
            if(set_mode_client.call(set_mode) && set_mode.response.mode_sent){
                ROS_INFO("AUTO enabled");
            }
            ros::Duration(1).sleep();
        }  
    }

    // Callback functions
    bool decoder_cb(cargo_drone::cmd::Request  &req,
        cargo_drone::cmd::Response &res){
        if (req.cmd == "MISSION1_START"){
            mission_thread = boost::thread(&DroneController::mission1_start, this);
        }
        else if (req.cmd == "CENTERING_START"){
            mission_thread = boost::thread(&DroneController::centering_start, this);
        }
        
        res.success = true;
        return true;
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    void relative_altitude_cb(const std_msgs::Float64::ConstPtr& msg){
        cur_rel_altitude = *msg;
        //std::cout << "altitude" << cur_rel_altitude.data << std::endl; 

    }

    void pose1_cb(const geometry_msgs::Pose::ConstPtr& msg){
        last_pose1_time = ros::Time::now().toSec();
        cur_pose_1 = *msg;
    }


};


int main(int argc, char **argv)
{

  // ROS node initialization
  ros::init(argc, argv, "drone_control");

  //boost::thread spin_thread(&spinThread);

  ros::NodeHandle nh;
  DroneController controller_node(&nh);
  
  //Thread initialization
  ros::spin();
  return 0;
 }