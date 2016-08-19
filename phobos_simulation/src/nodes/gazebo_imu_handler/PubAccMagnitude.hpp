#ifndef PUBACCMAGNITUDE_HPP_
#define PUBACCMAGNITUDE_HPP_

#include <ros/ros.h>
#include <std_msgs/Int16.h>

#include "GzIMU.hpp"

inline double sqr(double a){ return a*a;}

class PubAccMagnitude{
    ros::NodeHandle *nh;
    ros::Publisher pub;

    GzIMU* imu;
public:
    std_msgs::Int16 msg;

public:
    PubAccMagnitude( std::string topic, ros::NodeHandle *nh,GzIMU* imu){
        this->nh = nh;
        this->imu = imu;
        this->pub = nh->advertise<std_msgs::Int16>(topic, 100);
    }

    void Publish(){

        this->msg.data = sqr(imu->msg_imu.linear_acceleration.x) + sqr(imu->msg_imu.linear_acceleration.y) + sqr(imu->msg_imu.linear_acceleration.z);

        pub.publish(this->msg);
    }
};

#endif
