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
#include <dynamic_reconfigure/server.h>
#include <cargo_drone/drone_paramConfig.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/StreamRate.h>
#include <boost/thread.hpp>
#include <sstream>


class DroneController{
    private:
    ros::ServiceClient rate_client;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient takeoff_client;
    ros::ServiceServer service;
    ros::Subscriber state_sub;
    ros::Subscriber relative_altitude_sub;
    ros::Subscriber pose1_sub;
    ros::Subscriber mission_reached_sub;
    ros::Publisher vel_setpoint_pub;
    ros::Publisher raw_setpoint_pub;
    std_msgs::Float64 cur_rel_altitude;
    geometry_msgs::Pose cur_pose_1;
    geometry_msgs::TwistStamped vel_setpoint;
    mavros_msgs::State current_state;
    mavros_msgs::SetMode set_mode;
    mavros_msgs::PositionTarget vel_raw_data;
    mavros_msgs::StreamRate rate_data;

    double last_pose1_time;
    double vel_z;
    bool go_down;
    int mission_reached_checker;
    int mission_reached;
    int number_of_wp;
    
    boost::thread mission_thread;

    double x_kp,  y_kp, z_kp;
    double ar_timeout, diameter_thresh, lower_target_alt, 
            upper_target_alt, takeoff_alt, pid_limiter_xy,
            pid_limiter_z;
              
    //Setup dynamic reconfigure
    dynamic_reconfigure::Server<cargo_drone::drone_paramConfig> server;
    dynamic_reconfigure::Server<cargo_drone::drone_paramConfig>::CallbackType f;

    public:
    DroneController(ros::NodeHandle* nh)
    {   
        //immediately set rate to enable data retrieving from mavros topics (for APM)
        rate_client = nh->serviceClient<mavros_msgs::StreamRate>("/mavros/set_steram_rate");
        rate_data.request.stream_id = 0;
        rate_data.request.message_rate = 10;
        rate_data.request.on_off = 1;

        ROS_INFO("Waiting for set_stream_rate service...");

        if(ros::service::waitForService("/mavros/set_steram_rate", 10)){
            rate_client.call(rate_data);
            ROS_INFO("New stream rate is set");
        }else{
            ROS_INFO("set_stream_rate time out");
        }
        

        //Subscribers
        state_sub = nh->subscribe<mavros_msgs::State>("/mavros/state", 10, &DroneController::state_cb, this);
        relative_altitude_sub = nh->subscribe<std_msgs::Float64>("/mavros/global_position/rel_alt", 10, &DroneController::relative_altitude_cb, this);
        pose1_sub = nh->subscribe<geometry_msgs::Pose>("/aruco_simple/pose1", 10, &DroneController::pose1_cb, this);
        mission_reached_sub = nh->subscribe<mavros_msgs::WaypointReached>("/mavros/mission/reached", 10, &DroneController::mission_reached_cb, this);

        vel_setpoint_pub = nh->advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 100);
        raw_setpoint_pub = nh->advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

        //Services
        service = nh->advertiseService("/controller/drone_cmd", &DroneController::decoder_cb, this);

        arming_client = nh->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client = nh->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        takeoff_client = nh->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

        //non-nodehandle-correlated variables
        go_down = false;
        last_pose1_time = 0;
        mission_reached = 0;
        mission_reached_checker = 1;

        vel_raw_data.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        vel_raw_data.type_mask =
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_PX |
                mavros_msgs::PositionTarget::IGNORE_PY |
                mavros_msgs::PositionTarget::IGNORE_PZ |
                mavros_msgs::PositionTarget::IGNORE_YAW |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        f = boost::bind(&DroneController::dynamic_reconfigure_cb, this, _1, _2);
        server.setCallback(f);
    }

    void dynamic_reconfigure_cb(cargo_drone::drone_paramConfig &config, uint32_t level) {
        x_kp = config.x_kp;
        y_kp = config.y_kp;
        z_kp = config.z_kp;
        ar_timeout = config.ar_timeout;
        diameter_thresh = config.diameter_thresh;
        lower_target_alt = config.lower_target_alt;
        upper_target_alt = config.upper_target_alt;
        takeoff_alt = config.takeoff_alt;
        pid_limiter_xy = config.pid_limiter_xy;
        pid_limiter_z = config.pid_limiter_z;
        number_of_wp = config.number_of_wp;
    }

    void move_right(){
        vel_raw_data.velocity.x = 1;
        vel_raw_data.velocity.y = 0;

        vel_raw_data.header.stamp = ros::Time::now();
        raw_setpoint_pub.publish(vel_raw_data);
    }
    void move_left(){
        vel_raw_data.velocity.x = -1;
        vel_raw_data.velocity.y = 0;

        vel_raw_data.header.stamp = ros::Time::now();
        raw_setpoint_pub.publish(vel_raw_data);
    }
    void move_back(){
        vel_raw_data.velocity.x = 0;
        vel_raw_data.velocity.y = -1;

        vel_raw_data.header.stamp = ros::Time::now();
        raw_setpoint_pub.publish(vel_raw_data);
    }
    void move_front(){
        vel_raw_data.velocity.x = 0;
        vel_raw_data.velocity.y = 1;

        vel_raw_data.header.stamp = ros::Time::now();
        raw_setpoint_pub.publish(vel_raw_data);
    }

    double proporsional(double Kp, double setpoint, double state){
        return Kp*(state-setpoint);
    }

    void centering_start(){

        double current_time = ros::Time::now().toSec();
        //double duration = 5;
        set_mode.request.custom_mode = "GUIDED";
        while(current_state.mode != "GUIDED"){  
            ROS_INFO("Switching to GUIDED");
            if(set_mode_client.call(set_mode) && set_mode.response.mode_sent){
                ROS_INFO("GUIDED enabled");
                break;
            }
            ros::Duration(2).sleep();
        }

        ros::Rate send_vel_rate(1);   
        while(true){

            vel_z = proporsional(z_kp, lower_target_alt, cur_rel_altitude.data); 

            if(vel_z>pid_limiter_z){vel_z=pid_limiter_z;}
            if(vel_z<-pid_limiter_z){vel_z=-pid_limiter_z;}

            vel_raw_data.velocity.z = -vel_z;
            
            if (current_time - last_pose1_time < ar_timeout && last_pose1_time != 0){
                double vel_x = proporsional(x_kp, 0, cur_pose_1.position.x);
                double vel_y = proporsional(y_kp, 0, cur_pose_1.position.y);
                
                if(vel_x>pid_limiter_xy){vel_x=pid_limiter_xy;}
                if(vel_x<-pid_limiter_xy){vel_x=-pid_limiter_xy;}
                if(vel_y>pid_limiter_xy){vel_y=pid_limiter_xy;}
                if(vel_y<-pid_limiter_xy){vel_y=-pid_limiter_xy;}

                vel_raw_data.velocity.x = vel_x;
                vel_raw_data.velocity.y = -vel_y;
            }
            else{
                vel_raw_data.velocity.x = 0;
                vel_raw_data.velocity.y = 0;
            }

            raw_setpoint_pub.publish(vel_raw_data);

            if(cur_rel_altitude.data<=lower_target_alt+0.2){
                break;
            }

            send_vel_rate.sleep();
        } 

        go_back_up();
    }

    void go_back_up(){
        
        ros::Rate send_vel_rate(1);
        while(cur_rel_altitude.data < upper_target_alt*0.95){
            vel_z = proporsional(z_kp, upper_target_alt, cur_rel_altitude.data); 
            
            if(vel_z>pid_limiter_z){vel_z=pid_limiter_z;}
            if(vel_z<-pid_limiter_z){vel_z=-pid_limiter_z;}

            vel_raw_data.velocity.z = -vel_z;

            vel_raw_data.header.stamp = ros::Time::now();
            raw_setpoint_pub.publish(vel_raw_data);

            send_vel_rate.sleep();
        }
        
        //switch mode to RTL when final WP is reached
        if(mission_reached = number_of_wp){
            set_mode.request.custom_mode = "RTL";
            while(current_state.mode != "RTL"){
                if(set_mode_client.call(set_mode) && set_mode.response.mode_sent){
                    ROS_INFO("RTL enabled");
                }
            ros::Duration(1).sleep();
            }
        }else{
            //Switch to AUTO and wait until next wp is reached before going down
            set_mode.request.custom_mode = "AUTO";
            while(current_state.mode != "AUTO"){
                if(set_mode_client.call(set_mode) && set_mode.response.mode_sent){
                    ROS_INFO("AUTO enabled");
                }
            ros::Duration(1).sleep();
        }

        wait_wp_reached(); 
        }

        
    }
       
    void mission1_start(){

        //Set mode to GUIDED
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
        ros::Duration(6).sleep();

        //Take off
        mavros_msgs::CommandTOL takeoff_param;
        takeoff_param.request.altitude = takeoff_alt;

        if(takeoff_client.call(takeoff_param)){
            ROS_INFO("Taking off");
        }
         
        
        //wait until altitude is reached
        while(cur_rel_altitude.data < takeoff_alt*0.95){
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

        wait_wp_reached();    
        ROS_INFO("MISSION 1 END");
    }

    void wait_wp_reached(){
        while(true){
            if(mission_reached_checker == mission_reached){    
                mission_reached_checker++;
                centering_start();
            }
            ros::Duration(0.5).sleep();
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
        else if (req.cmd == "MOVE_LEFT"){
            mission_thread = boost::thread(&DroneController::move_left, this);
        }
        else if (req.cmd == "MOVE_RIGHT"){
            mission_thread = boost::thread(&DroneController::move_right, this);
        }
        else if (req.cmd == "MOVE_BACK"){
            mission_thread = boost::thread(&DroneController::move_back, this);
        }
        else if (req.cmd == "MOVE_FRONT"){
            mission_thread = boost::thread(&DroneController::move_front, this);
        }
        
        res.success = true;
        return true;
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    void relative_altitude_cb(const std_msgs::Float64::ConstPtr& msg){
        cur_rel_altitude = *msg;
    }

    void pose1_cb(const geometry_msgs::Pose::ConstPtr& msg){
        last_pose1_time = ros::Time::now().toSec();
        cur_pose_1 = *msg;
    }

    void mission_reached_cb(const mavros_msgs::WaypointReached::ConstPtr& msg){
        mission_reached = msg->wp_seq;
    }

};

int main(int argc, char **argv)
{
  // ROS node initialization
  ros::init(argc, argv, "drone_control");
  
  ros::NodeHandle nh;
  DroneController controller_node(&nh);
  
  ros::spin();
  return 0;
 }