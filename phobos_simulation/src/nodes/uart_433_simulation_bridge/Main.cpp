#include <ros/ros.h>

#include <phobos_shared/EncodersArm.h>
#include <phobos_shared/EncodersWheels.h>
#include <phobos_shared/EncodersRockerBogie.h>

#include "../../../../../../phobos_control/src/phobos_control_station/src/uart_433_control_transceiver/KeyDefinitions.hpp"
#include "../../../../../../phobos_control/src/phobos_control_station/src/uart_433_control_transceiver/PubJointsState.hpp"
#include "../../../../../../phobos_control/src/phobos_control_station/src/uart_433_control_transceiver/PubPoseOrient.hpp"
#include "../../../../../../phobos_control/src/phobos_control_station/src/uart_433_control_transceiver/Status.hpp"
#include "../../../../../../phobos_control/src/phobos_control_station/src/uart_433_control_transceiver/SubKey.hpp"

#include "../../../../../../phobos_rover/src/phobos_rover_communication/src/uart_433_rover_transceiver/PubArmVel.hpp"
#include "../../../../../../phobos_rover/src/phobos_rover_communication/src/uart_433_rover_transceiver/PubWheelsVel.hpp"
#include "../../../../../../phobos_rover/src/phobos_rover_communication/src/uart_433_rover_transceiver/SubEncoders.hpp"
#include "../../../../../../phobos_rover/src/phobos_rover_communication/src/uart_433_rover_transceiver/SubError.hpp"
#include "../../../../../../phobos_rover/src/phobos_rover_communication/src/uart_433_rover_transceiver/SubJointsState.hpp"
#include "../../../../../../phobos_rover/src/phobos_rover_communication/src/uart_433_rover_transceiver/SubOdom.hpp"


int main(int argc, char** argv){

    ros::init(argc, argv, "uart_433_simulation_bridge");
    ros::NodeHandle nh;

    int BASIC_RATE;

    nh.param("basic_rate", BASIC_RATE, 5);

// CONTROL
    Status __status;

    SubKey sub_key(&nh, &__status);
    // SubJoy sub_joy(&nh, &__status);

    PubPoseOrient pose_orient(&nh, "/control/localization/pose_ekf");
    PubJointsState joints_state(&nh, "/control/encoders/joints_state");

// ROVER
    SubOdom odom("/rover/localization/odom_ekf", &nh);
    SubError error_control("/rover/security/error_code", &nh);
    SubEncoders <phobos_shared::EncodersArm> arm_encoders ("/rover/encoders/arm_absolute", &nh);
    SubEncoders <phobos_shared::EncodersWheels> wheels_encoders ("/rover/encoders/wheels_relative", &nh);
    SubEncoders <phobos_shared::EncodersRockerBogie> rocker_bogie_encoders ("/rover/encoders/rocker_bogie_absolute", &nh);

    PubWheelsVel wheels_vel("/rover/control/wheels_vel", &nh);
    PubArmVel arm_vel("/rover/control/arm_vel", &nh);


    ros::Rate loop_rate(BASIC_RATE);

    while(ros::ok()){
        ros::spinOnce();

// ROVER
        wheels_vel.msg.wheels_left = __status.wheels_left;
        wheels_vel.msg.wheels_right = __status.wheels_right;
        wheels_vel.Publish();

        arm_vel.msg.link_0 = __status.link_0;
        arm_vel.msg.link_1 = __status.link_1;
        arm_vel.msg.link_2 = __status.link_2;
        arm_vel.msg.link_3 = __status.link_3;
        arm_vel.msg.link_4 = __status.link_4;
        arm_vel.msg.grip_force  = __status.grip_force;
        arm_vel.Publish();

// CONTROL
        pose_orient.msg_pose.header.seq++;
        pose_orient.msg_pose.header.stamp = ros::Time::now();
        pose_orient.msg_pose.pose.position.x = odom.msg.pose.pose.position.x;
        pose_orient.msg_pose.pose.position.y = odom.msg.pose.pose.position.y;
        pose_orient.msg_pose.pose.position.z = odom.msg.pose.pose.position.z;
        pose_orient.msg_pose.pose.orientation.x = odom.msg.pose.pose.orientation.x;
        pose_orient.msg_pose.pose.orientation.y = odom.msg.pose.pose.orientation.y;
        pose_orient.msg_pose.pose.orientation.z = odom.msg.pose.pose.orientation.z;
        pose_orient.msg_pose.pose.orientation.w = odom.msg.pose.pose.orientation.w;
        pose_orient.Publish();

        joints_state.msg_joints.wheel_vel_fl = wheels_encoders.msg.wheel_vel_fl;
        joints_state.msg_joints.wheel_vel_fr = wheels_encoders.msg.wheel_vel_fr;
        joints_state.msg_joints.wheel_vel_ml = wheels_encoders.msg.wheel_vel_ml;
        joints_state.msg_joints.wheel_vel_mr = wheels_encoders.msg.wheel_vel_mr;
        joints_state.msg_joints.wheel_vel_bl = wheels_encoders.msg.wheel_vel_bl;
        joints_state.msg_joints.wheel_vel_br = wheels_encoders.msg.wheel_vel_br;
        joints_state.msg_joints.link_pose_0 = arm_encoders.msg.link_pose_0;
        joints_state.msg_joints.link_pose_1 = arm_encoders.msg.link_pose_1;
        joints_state.msg_joints.link_pose_2 = arm_encoders.msg.link_pose_2;
        joints_state.msg_joints.link_pose_3 = arm_encoders.msg.link_pose_3;
        joints_state.msg_joints.link_pose_4 = arm_encoders.msg.link_pose_4;
        joints_state.msg_joints.grip_pose = arm_encoders.msg.grip_pose;
        joints_state.msg_joints.rocker_pose_l = rocker_bogie_encoders.msg.rocker_pose_l;
        joints_state.msg_joints.rocker_pose_r = rocker_bogie_encoders.msg.rocker_pose_r;
        joints_state.msg_joints.bogie_pose_l = rocker_bogie_encoders.msg.bogie_pose_l;
        joints_state.msg_joints.bogie_pose_r = rocker_bogie_encoders.msg.bogie_pose_r;
        joints_state.Publish();

        loop_rate.sleep();
    }
    return 0;
}
