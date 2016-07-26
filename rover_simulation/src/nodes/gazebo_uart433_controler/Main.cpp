#include <ros/ros.h>

#include "PubGazeboJointVel.hpp"
#include "SubArmVel.hpp"
#include "SubWheelVel.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "gazebo_uart433_controler");
    ros::NodeHandle nh;

    PubGazeboJointVel pub_joint_vel(&nh);

    SubWheelVel sub_wheel_vel("/rover/control/wheels_vel", &nh, &pub_joint_vel);
    SubArmVel sub_arm_vel("/rover/control/arm_vel", &nh, &pub_joint_vel);

    ros::spin();

    return 0;
}
