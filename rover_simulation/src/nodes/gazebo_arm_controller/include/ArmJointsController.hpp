#ifndef ARMCONTROLLERSIMPLE_HPP_
#define ARMCONTROLLERSIMPLE_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "ControllerPID.hpp"


class ArmJointsController
{
protected:
    std_msgs::Float64 temp_msg;

    ros::NodeHandle nh;

    ros::Publisher pub_joint_cmd[9];
    ros::Subscriber sub_joint_cmd[9];
    ros::Subscriber sub_joint_pose[9];

    double joint_pose[9];
    double recived_pose[9];

    ControllerPID pid[9];

public:
    ArmJointsController();
    ~ArmJointsController();

    void SetJointVelocity(int i, double velocity)
    {
        temp_msg.data = velocity;
        pub_joint_cmd[i].publish(temp_msg);
    }

    void UpdateVelocity(int num, double joint_pose, double recived_pose, double precision)
    {
        if(fabs(joint_pose - recived_pose) < precision)
            SetJointVelocity(num, 0);
        else
            SetJointVelocity(num, pid[num].Controll(joint_pose, recived_pose));

    }

    void PublishCmds()
    {

        UpdateVelocity(0, joint_pose[0], recived_pose[0], 0.05);
        UpdateVelocity(1, joint_pose[1], recived_pose[1], 0.05);
        UpdateVelocity(2, joint_pose[2], recived_pose[2], 0.05);
        UpdateVelocity(3, joint_pose[3], recived_pose[3], 0.05);
        UpdateVelocity(4, joint_pose[4], recived_pose[4], 0.05);
        UpdateVelocity(5, joint_pose[5], recived_pose[5], 0.005);
        UpdateVelocity(6, joint_pose[6], recived_pose[6], 0.005);
        //UpdateVelocity(7, joint_pose[7], recived_pose[7], 0.1);
        //UpdateVelocity(8, joint_pose[8], recived_pose[8], 0.1);
    }

    void RecivedLinkCmd0(const std_msgs::Float64 recived_pose)
    {
        this->recived_pose[0] = recived_pose.data;
    }

    void RecivedLinkCmd1(const std_msgs::Float64 recived_pose)
    {
        this->recived_pose[1] = recived_pose.data;
    }

    void RecivedLinkCmd2(const std_msgs::Float64 recived_pose)
    {
        this->recived_pose[2] = recived_pose.data;
    }

    void RecivedLinkCmd3(const std_msgs::Float64 recived_pose)
    {
        this->recived_pose[3] = recived_pose.data;
    }

    void RecivedLinkCmd4(const std_msgs::Float64 recived_pose)
    {
        this->recived_pose[4] = recived_pose.data;
    }

    void RecivedFingerCmd1(const std_msgs::Float64 recived_pose)
    {
        this->recived_pose[5] = recived_pose.data;
    }

    void RecivedFingerCmd2(const std_msgs::Float64 recived_pose)
    {
        this->recived_pose[6] = recived_pose.data;
    }

    void RecivedTowerYawCmd(const std_msgs::Float64 recived_pose)
    {
        this->recived_pose[7] = recived_pose.data;
    }

    void RecivedTowerPitchCmd(const std_msgs::Float64 recived_pose)
    {
        this->recived_pose[8] = recived_pose.data;
    }



    void RecivedLinkPose0(const std_msgs::Float64 recived_pose)
    {
        this->joint_pose[0] = recived_pose.data;
    }

    void RecivedLinkPose1(const std_msgs::Float64 recived_pose)
    {
        this->joint_pose[1] = recived_pose.data;
    }

    void RecivedLinkPose2(const std_msgs::Float64 recived_pose)
    {
        this->joint_pose[2] = recived_pose.data;
    }

    void RecivedLinkPose3(const std_msgs::Float64 recived_pose)
    {
        this->joint_pose[3] = recived_pose.data;
    }

    void RecivedLinkPose4(const std_msgs::Float64 recived_pose)
    {
        this->joint_pose[4] = recived_pose.data;
    }

    void RecivedFingerPose1(const std_msgs::Float64 recived_pose)
    {
        this->joint_pose[5] = recived_pose.data;
    }

    void RecivedFingerPose2(const std_msgs::Float64 recived_pose)
    {
        this->joint_pose[6] = recived_pose.data;
    }

    void RecivedTowerYawPose(const std_msgs::Float64 recived_pose)
    {
        this->joint_pose[7] = recived_pose.data;
    }

    void RecivedTowerPitchPose(const std_msgs::Float64 recived_pose)
    {
        this->joint_pose[8] = recived_pose.data;
    }

};

#endif
