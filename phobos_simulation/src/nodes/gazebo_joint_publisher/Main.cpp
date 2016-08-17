#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

#include "GzJointState.hpp"
#include "AddJoint.hpp"
#include "JointStatesPublisher.hpp"

#define RATE 10

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "gazebo_joint_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(RATE);

    JointStatesPublisher joint_state_publisher;

    joint_state_publisher.Add("link_0_joint");
    joint_state_publisher.Add("link_1_joint");
    joint_state_publisher.Add("link_2_joint");
    joint_state_publisher.Add("link_3_joint");
    joint_state_publisher.Add("link_4_joint");
    joint_state_publisher.Add("finger1_joint");
    joint_state_publisher.Add("finger2_joint");
    // joint_state_publisher.Add("tower_yaw_joint");
    // joint_state_publisher.Add("tower_pitch_joint");
    joint_state_publisher.Add("wheel_bl_joint");
    joint_state_publisher.Add("wheel_ml_joint");
    joint_state_publisher.Add("wheel_fl_joint");
    joint_state_publisher.Add("wheel_br_joint");
    joint_state_publisher.Add("wheel_mr_joint");
    joint_state_publisher.Add("wheel_fr_joint");
    joint_state_publisher.Add("rocker_l_bearing_joint");
    joint_state_publisher.Add("rocker_r_bearing_joint");
    joint_state_publisher.Add("bogie_l_bearing_joint");
    joint_state_publisher.Add("bogie_r_bearing_joint");

    while(ros::ok()){
        joint_state_publisher.Update();
        loop_rate.sleep();
    }

    return 0;
}
