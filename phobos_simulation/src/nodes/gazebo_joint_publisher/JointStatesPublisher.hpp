#ifndef JOINTSTATESPUBLISHER_HPP_
#define JOINTSTATESPUBLISHER_HPP_

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

#include "AddJoint.hpp"
#include "GzJointState.hpp"

class JointStatesPublisher{
    ros::NodeHandle nh;
    ros::Publisher pub_js;

    std::vector<AddJoint> joints_vec;

protected:
    sensor_msgs::JointState msg_js;

public:
    JointStatesPublisher(){
        pub_js = nh.advertise<sensor_msgs::JointState>("rover/joint_states", 100);

        sensor_msgs::JointState __msg_js;
        msg_js.header.seq = 0;
        msg_js.header.stamp = ros::Time::now();
        msg_js.header.frame_id = "phobos_js";
    }

    void Add(std::string joint_name){
        joints_vec.push_back(AddJoint(&this->msg_js, joint_name));
    }

    void Update(){
        msg_js.header.seq++;
        msg_js.header.stamp = ros::Time::now();
        if(joints_vec.size() > 0){
            for(int i = 0; i < joints_vec.size(); i++){
                joints_vec[i].Update();
            }
        }

        pub_js.publish(msg_js);
    }
};

#endif
