#ifndef GZLINKSTATE_HPP_
#define GZLINKSTATE_HPP_

#include <ros/ros.h>
#include <string>

#include "gazebo_msgs/GetLinkState.h"

class GzLinkState{
private:
    ros::NodeHandle nh;
    ros::ServiceClient service_client;
protected:
    gazebo_msgs::GetLinkState msg_link_state;

public:
    GzLinkState(std::string link_name){
        service_client = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
        msg_link_state.request.link_name = link_name;
    }
    ~GzLinkState(){
        //Nothing :)
    }

    void Update(){
        if(!service_client.call(msg_link_state)) ROS_ERROR("Unable to get link state: IMU");

        // double x = msg_link_state.response.link_state.pose.position.x;
        // double y = msg_link_state.response.link_state.pose.position.y;
        // double z = msg_link_state.response.link_state.pose.position.z;
        //
        // ROS_INFO("%f %f %f", x,y,z);
    }
};

#endif
