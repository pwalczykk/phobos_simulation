#ifndef BASEWHEELCONTROLLER_HPP_
#define BASEWHEELCONTROLLER_HPP_

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

class BaseWheelController{
    std_msgs::Float64 temp_msg;

    std_msgs::Float64 left_wheels_speed;
    std_msgs::Float64 right_wheels_speed;

    std_msgs::Float64 left_wheels_speed_reduced;
    std_msgs::Float64 right_wheels_speed_reduced;

    std_msgs::Float64 wheel_stop;
    std_msgs::Float64 wheel_free;

    ros::NodeHandle nh;

    ros::Subscriber sub_vel;
    ros::Publisher pub_wheel[6];
public:
    BaseWheelController();
    ~BaseWheelController();

    void SubCmdVel(const geometry_msgs::Twist cmd_vel);
    void PubWheelSpeed();
};

#endif
