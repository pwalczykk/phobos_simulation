#ifndef PUBLISHERTWIST_HPP_
#define PUBLISHERTWIST_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class PublisherTwist
{
    geometry_msgs::Twist cmd_vel;

    ros::NodeHandle* nh;
    ros::Publisher pub;

public:
    PublisherTwist(){};
    PublisherTwist(const char* topic, ros::NodeHandle* nh);
    ~PublisherTwist(){};

    void Init(const char* topic, ros::NodeHandle* nh);

    void Publish(double linear_x, double angular_z);
};

#endif
