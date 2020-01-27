/****************************************************************************
* Software License Agreement (Apache License)
*
*     Copyright (C) 2012-2013 Open Source Robotics Foundation
*
*     Licensed under the Apache License, Version 2.0 (the "License");
*     you may not use this file except in compliance with the License.
*     You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
*     Unless required by applicable law or agreed to in writing, software
*     distributed under the License is distributed on an "AS IS" BASIS,
*     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*     See the License for the specific language governing permissions and
*     limitations under the License.
*
*****************************************************************************/

#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <mavros_msgs/State.h>
#include <boost/lexical_cast.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/videoio.hpp>
#endif
#include <ctime>

std::string datetime()
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%d-%m-%Y %H-%M-%S",timeinfo);
    return std::string(buffer);
}

cv::VideoWriter outputVideo;

int g_count = 0;
int g_last_wrote_time = 0;
std::string encoding;
std::string codec;
std::string filename;
std::string compressed;
double min_depth_range;
double max_depth_range;
bool use_dynamic_range;
int colormap;
bool prev_armed_state;
bool is_armed;
int penamaan = 1;
int fps;

void open_output_video(){
    cv::Size size(640, 480);
            outputVideo.open(filename, 
#if CV_MAJOR_VERSION >= 3
            cv::VideoWriter::fourcc(codec.c_str()[0],
#else
            CV_FOURCC(codec.c_str()[0],
#endif
                        codec.c_str()[1],
                        codec.c_str()[2],
                        codec.c_str()[3]), 
            30,
            size,
            true);
}

void callback(const sensor_msgs::ImageConstPtr& image_msg)
{   

    if (!outputVideo.isOpened() || !is_armed)
    {
       return;
    }
        
    try
    {
      cv_bridge::CvtColorForDisplayOptions options;
      options.min_image_value = 0.0;
      options.max_image_value = 0.0;
      options.colormap = -1;
      const cv::Mat image = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg), encoding, options)->image;
      if (!image.empty()) {
        outputVideo << image;
        ROS_INFO_STREAM("Recording frame " << g_count << "\x1b[1F");
        g_count++;
        g_last_wrote_time = ros::Time::now().nsec;
      } else {
          ROS_WARN("Frame skipped, no data!");
      }
    } catch(cv_bridge::Exception)
    {
        ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
        return;
    }
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    is_armed = msg->armed;

    if (prev_armed_state == false && is_armed == true){
        open_output_video();
    } else if(prev_armed_state == true && is_armed == false){
        filename = datetime() + ".avi";
    }
    prev_armed_state = is_armed;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_recorder", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");
    local_nh.param("filename", filename, datetime());
    bool stamped_filename;
    local_nh.param("stamped_filename", stamped_filename, true);
    local_nh.param("codec", codec, std::string("MJPG"));
    local_nh.param("encoding", encoding, std::string("bgr8"));
    local_nh.param("image_transport", compressed, std::string("compressed"));

    // cv_bridge::CvtColorForDisplayOptions

    if (codec.size() != 4) {
        ROS_ERROR("The video codec must be a FOURCC identifier (4 chars)");
        exit(-1);
    }

    image_transport::ImageTransport it(nh);
    std::string topic = "/aruco_simple/result";
    image_transport::Subscriber sub_image = it.subscribe(topic, 1, callback);
    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 100, state_cb);
    

    ROS_INFO_STREAM("Waiting for topic " << topic << "...");
    ros::spin();
}
