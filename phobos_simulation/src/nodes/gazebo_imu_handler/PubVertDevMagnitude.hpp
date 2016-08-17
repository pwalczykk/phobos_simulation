#ifndef PUBVERTDEVMAGNITUDE_HPP_
#define PUBVERTDEVMAGNITUDE_HPP_

#include <ros/ros.h>
#include <std_msgs/Int16.h>

#include "GzIMU.hpp"

class PubVertDevMagnitude{
    ros::NodeHandle *nh;
    ros::Publisher pub;

    GzIMU* imu;
public:
    std_msgs::Int16 msg;

public:
    PubVertDevMagnitude(std::string topic, ros::NodeHandle *nh, GzIMU* imu){
        this->nh = nh;
        this->imu = imu;
        this->pub = nh->advertise<geometry_msgs::PoseStamped>(topic, 100);
    }

    void Publish(){
        // TODO - vertical deviation
        this->msg.data = 12345;
        pub.publish(this->msg);
    }
};

#endif
