#ifndef PUBGAZEBOJOINTVEL_HPP_
#define PUBGAZEBOJOINTVEL_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64.h>

class PubGazeboJointVel{
    std_msgs::Float64 temp_msg;

    ros::NodeHandle *nh;

    ros::Publisher pub_joint[13];
    double joint_vel[13];
public:
    PubGazeboJointVel(ros::NodeHandle *nh){
        this->nh = nh;

        pub_joint[0] = nh->advertise<std_msgs::Float64>("/rover/link0_velocity_controller/command", 100);
        pub_joint[1] = nh->advertise<std_msgs::Float64>("/rover/link1_velocity_controller/command", 100);
        pub_joint[2] = nh->advertise<std_msgs::Float64>("/rover/link2_velocity_controller/command", 100);
        pub_joint[3] = nh->advertise<std_msgs::Float64>("/rover/link3_velocity_controller/command", 100);
        pub_joint[4] = nh->advertise<std_msgs::Float64>("/rover/link4_velocity_controller/command", 100);

        pub_joint[5] = nh->advertise<std_msgs::Float64>("/rover/finger1_velocity_controller/command", 100);
        pub_joint[6] = nh->advertise<std_msgs::Float64>("/rover/finger2_velocity_controller/command", 100);

        pub_joint[7] = nh->advertise<std_msgs::Float64>("rover/wheel_fl_velocity_controller/command", 100);
        pub_joint[8] = nh->advertise<std_msgs::Float64>("rover/wheel_fr_velocity_controller/command", 100);
        pub_joint[9] = nh->advertise<std_msgs::Float64>("rover/wheel_ml_velocity_controller/command", 100);
        pub_joint[10] = nh->advertise<std_msgs::Float64>("rover/wheel_mr_velocity_controller/command", 100);
        pub_joint[11] = nh->advertise<std_msgs::Float64>("rover/wheel_bl_velocity_controller/command", 100);
        pub_joint[12] = nh->advertise<std_msgs::Float64>("rover/wheel_br_velocity_controller/command", 100);

        for(int i = 0; i < 13; i++){
            joint_vel[i] = 0;
        }
    }

    void UpdateVel(int i, double vel){
        if(joint_vel[i] != vel){
            temp_msg.data = vel;
            pub_joint[i].publish(temp_msg);
            joint_vel[i] = vel;
        }
    }


};

#endif
