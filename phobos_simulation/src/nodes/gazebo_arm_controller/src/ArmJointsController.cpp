#include "../include/ArmJointsController.hpp"
#include "../include/ControllerPID.hpp"

ArmJointsController::ArmJointsController()
{
    pub_joint_cmd[0] = nh.advertise<std_msgs::Float64>("/rover/link_0_velocity_controller/command", 100);
    pub_joint_cmd[1] = nh.advertise<std_msgs::Float64>("/rover/link_1_velocity_controller/command", 100);
    pub_joint_cmd[2] = nh.advertise<std_msgs::Float64>("/rover/link_2_velocity_controller/command", 100);
    pub_joint_cmd[3] = nh.advertise<std_msgs::Float64>("/rover/link_3_velocity_controller/command", 100);
    pub_joint_cmd[4] = nh.advertise<std_msgs::Float64>("/rover/link_4_velocity_controller/command", 100);

    pub_joint_cmd[5] = nh.advertise<std_msgs::Float64>("/rover/finger1_velocity_controller/command", 100);
    pub_joint_cmd[6] = nh.advertise<std_msgs::Float64>("/rover/finger2_velocity_controller/command", 100);

    pub_joint_cmd[7] = nh.advertise<std_msgs::Float64>("/rover/tower_base_velocity_controller/command", 100);
    pub_joint_cmd[8] = nh.advertise<std_msgs::Float64>("/rover/tower_head_velocity_controller/command", 100);

    sub_joint_cmd[0] = nh.subscribe("/rover/link_0_position_controller/command", 100, &ArmJointsController::RecivedLinkCmd0, this);
    sub_joint_cmd[1] = nh.subscribe("/rover/link_1_position_controller/command", 100, &ArmJointsController::RecivedLinkCmd1, this);
    sub_joint_cmd[2] = nh.subscribe("/rover/link_2_position_controller/command", 100, &ArmJointsController::RecivedLinkCmd2, this);
    sub_joint_cmd[3] = nh.subscribe("/rover/link_3_position_controller/command", 100, &ArmJointsController::RecivedLinkCmd3, this);
    sub_joint_cmd[4] = nh.subscribe("/rover/link_4_position_controller/command", 100, &ArmJointsController::RecivedLinkCmd4, this);

    sub_joint_cmd[5] = nh.subscribe("/rover/finger1_position_controller/command", 100, &ArmJointsController::RecivedFingerCmd1, this);
    sub_joint_cmd[6] = nh.subscribe("/rover/finger2_position_controller/command", 100, &ArmJointsController::RecivedFingerCmd2, this);

    sub_joint_cmd[7] = nh.subscribe("/rover/tower_yaw_position_controller/command", 100, &ArmJointsController::RecivedTowerYawCmd, this);
    sub_joint_cmd[8] = nh.subscribe("/rover/tower_pitch_position_controller/command", 100, &ArmJointsController::RecivedTowerPitchCmd, this);

    sub_joint_pose[0] = nh.subscribe("/rover/link_0_velocity_controller/pose", 100, &ArmJointsController::RecivedLinkPose0, this);
    sub_joint_pose[1] = nh.subscribe("/rover/link_1_velocity_controller/pose", 100, &ArmJointsController::RecivedLinkPose1, this);
    sub_joint_pose[2] = nh.subscribe("/rover/link_2_velocity_controller/pose", 100, &ArmJointsController::RecivedLinkPose2, this);
    sub_joint_pose[3] = nh.subscribe("/rover/link_3_velocity_controller/pose", 100, &ArmJointsController::RecivedLinkPose3, this);
    sub_joint_pose[4] = nh.subscribe("/rover/link_4_velocity_controller/pose", 100, &ArmJointsController::RecivedLinkPose4, this);

    sub_joint_pose[5] = nh.subscribe("/rover/finger1_velocity_controller/pose", 100, &ArmJointsController::RecivedFingerPose1, this);
    sub_joint_pose[6] = nh.subscribe("/rover/finger2_velocity_controller/pose", 100, &ArmJointsController::RecivedFingerPose2, this);

    sub_joint_pose[7] = nh.subscribe("/rover/tower_yaw_velocity_controller/pose", 100, &ArmJointsController::RecivedTowerYawPose, this);
    sub_joint_pose[8] = nh.subscribe("/rover/tower_pitch_velocity_controller/pose", 100, &ArmJointsController::RecivedTowerPitchPose, this);


    pid[0].Set(0, 0, 0);
    pid[1].Set(0, 0, 0);
    pid[2].Set(0, 0, 0);
    pid[3].Set(0, 0, 0);
    pid[4].Set(0, 0, 0);
    pid[5].Set(0, 0, 0);
    pid[6].Set(0, 0, 0);
    pid[7].Set(0, 0, 0);
    pid[8].Set(0, 0, 0);

    for(int i = 0; i < 9; i++)
    {
        joint_pose[i] = 0;
        recived_pose[i] = 0;
    }
}

ArmJointsController::~ArmJointsController()
{}
