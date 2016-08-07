#include <ros/ros.h>
#include "../include/BaseWheelController.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_wheel_controller");

    BaseWheelController base_wheel_controller;

    ros::spin();
}
