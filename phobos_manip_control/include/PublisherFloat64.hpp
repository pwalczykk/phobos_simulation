#ifndef PUBLISHERFLOAT64_HPP_
#define PUBLISHERFLOAT64_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64.h>

class PublisherFloat64
{
    double current_value;

    std_msgs::Float64 msg;

    ros::NodeHandle* nh;
    ros::Publisher pub;

public:
    PublisherFloat64(){};
    PublisherFloat64(const char* topic, ros::NodeHandle* nh);
    ~PublisherFloat64(){};

    void Init(const char* topic, ros::NodeHandle* nh);

    void Publish(double msg_data);
};

#endif
