#ifndef GZJOINTSTATE_HPP_
#define GZJOINTSTATE_HPP_

#include <ros/ros.h>
#include <string>

#include "gazebo_msgs/GetJointProperties.h"

class GzJointState{
private:
    ros::NodeHandle nh;
    ros::ServiceClient service_client;

protected:
    gazebo_msgs::GetJointProperties msg_joint_properties;

public:
    GzJointState(std::string joint_name){
        service_client = nh.serviceClient<gazebo_msgs::GetJointProperties>("gazebo/get_joint_properties");
        msg_joint_properties.request.joint_name = joint_name;

    }

    void Update(){
        if(!service_client.call(msg_joint_properties)) ROS_ERROR("Unable to get joint state");
    }
};

#endif
