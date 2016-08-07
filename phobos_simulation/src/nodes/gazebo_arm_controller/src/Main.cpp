#include <ros/ros.h>
#include "../include/ArmJointDRC.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_sim_controller");

    //ArmJointsController arm_jonit_controller;

    ArmJointDRC arm_jonit_drc;

    ros::Rate loop_rate(10);

    while(ros::ok){
        arm_jonit_drc.PublishCmds();
        ros::spinOnce();
        loop_rate.sleep();
    }

}
