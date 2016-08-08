#include "../include/PublisherTwist.hpp"

PublisherTwist::PublisherTwist(const char* topic, ros::NodeHandle* nh)
{
    Init(topic, nh);
}

void PublisherTwist::Init(const char* topic, ros::NodeHandle* nh)
{
    pub = nh->advertise<geometry_msgs::Twist>(topic, 100);
    Publish(0, 0);
}

void PublisherTwist::Publish(double linear_x, double angular_z)
{
    cmd_vel.linear.x = linear_x;
    cmd_vel.angular.z = angular_z;
    pub.publish(cmd_vel);
}
