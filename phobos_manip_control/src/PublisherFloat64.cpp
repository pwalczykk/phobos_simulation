#include "../include/PublisherFloat64.hpp"

PublisherFloat64::PublisherFloat64(const char* topic, ros::NodeHandle* nh)
{
    Init(topic, nh);
}

void PublisherFloat64::Init(const char* topic, ros::NodeHandle* nh)
{
    pub = nh->advertise<std_msgs::Float64>(topic, 100);
    Publish(0);
}

void PublisherFloat64::Publish(double msg_data)
{
    msg.data = msg_data;
    pub.publish(msg);
}
