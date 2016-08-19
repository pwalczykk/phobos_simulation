#include <ros/ros.h>

#include <phobos_shared/EncodersArm.h>
#include <phobos_shared/EncodersWheels.h>
#include <phobos_shared/EncodersRockerBogie.h>
#include <phobos_shared/EncodersAll.h>

#include "../../../../../../phobos_shared/src/phobos_shared/include/JointConfig.hpp"

#include "../../../../../../phobos_control/src/phobos_control_station/src/uart_433_control_transceiver/AddJoint.hpp"
#include "../../../../../../phobos_control/src/phobos_control_station/src/uart_433_control_transceiver/KeyDefinitions.hpp"
#include "../../../../../../phobos_control/src/phobos_control_station/src/uart_433_control_transceiver/PubEncodersAll.hpp"
#include "../../../../../../phobos_control/src/phobos_control_station/src/uart_433_control_transceiver/PubErrorCode.hpp"
#include "../../../../../../phobos_control/src/phobos_control_station/src/uart_433_control_transceiver/PubJointState.hpp"
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

    PubErrorCode security_error(&nh, "/control/security/error_code");
    PubPoseOrient pose_orient(&nh, "/control/localization/pose_ekf");
    PubEncodersAll encoders_all(&nh, "/control/encoders/all");
    PubJointState joint_state(&nh, "/control/encoders/joint_state");
        joint_state.Add("link_0_joint");
        joint_state.Add("link_1_joint");
        joint_state.Add("link_2_joint");
        joint_state.Add("link_3_joint");
        joint_state.Add("link_4_joint");
        joint_state.Add("finger_l_joint");
        joint_state.Add("finger_r_joint");
        joint_state.Add("rocker_l_bearing_joint");
        joint_state.Add("rocker_r_bearing_joint");
        joint_state.Add("bogie_l_bearing_joint");
        joint_state.Add("bogie_r_bearing_joint");
        joint_state.Add("wheel_bl_joint");
        joint_state.Add("wheel_ml_joint");
        joint_state.Add("wheel_fl_joint");
        joint_state.Add("wheel_br_joint");
        joint_state.Add("wheel_mr_joint");
        joint_state.Add("wheel_fr_joint");

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

        encoders_all.msg_encoders.wheel_vel_fl = wheels_encoders.msg.wheel_vel_fl;
        encoders_all.msg_encoders.wheel_vel_fr = wheels_encoders.msg.wheel_vel_fr;
        encoders_all.msg_encoders.wheel_vel_ml = wheels_encoders.msg.wheel_vel_ml;
        encoders_all.msg_encoders.wheel_vel_mr = wheels_encoders.msg.wheel_vel_mr;
        encoders_all.msg_encoders.wheel_vel_bl = wheels_encoders.msg.wheel_vel_bl;
        encoders_all.msg_encoders.wheel_vel_br = wheels_encoders.msg.wheel_vel_br;
        encoders_all.msg_encoders.link_pose_0 = arm_encoders.msg.link_pose_0;
        encoders_all.msg_encoders.link_pose_1 = arm_encoders.msg.link_pose_1;
        encoders_all.msg_encoders.link_pose_2 = arm_encoders.msg.link_pose_2;
        encoders_all.msg_encoders.link_pose_3 = arm_encoders.msg.link_pose_3;
        encoders_all.msg_encoders.link_pose_4 = arm_encoders.msg.link_pose_4;
        encoders_all.msg_encoders.grip_pose = arm_encoders.msg.grip_pose;
        encoders_all.msg_encoders.rocker_pose_l = rocker_bogie_encoders.msg.rocker_pose_l;
        encoders_all.msg_encoders.rocker_pose_r = rocker_bogie_encoders.msg.rocker_pose_r;
        encoders_all.msg_encoders.bogie_pose_l = rocker_bogie_encoders.msg.bogie_pose_l;
        encoders_all.msg_encoders.bogie_pose_r = rocker_bogie_encoders.msg.bogie_pose_r;
        encoders_all.Publish();

        joint_state.msg_js.header.seq++;
        joint_state.msg_js.header.stamp = ros::Time::now();
        joint_state.msg_js.position[0] = arm_encoders.msg.link_pose_0;
        joint_state.msg_js.position[1] = arm_encoders.msg.link_pose_1;
        joint_state.msg_js.position[2] = arm_encoders.msg.link_pose_2;
        joint_state.msg_js.position[3] = arm_encoders.msg.link_pose_3;
        joint_state.msg_js.position[4] = arm_encoders.msg.link_pose_4;
        joint_state.msg_js.position[5] = arm_encoders.msg.grip_pose;
        joint_state.msg_js.position[6] = arm_encoders.msg.grip_pose;
        joint_state.msg_js.position[7] = rocker_bogie_encoders.msg.rocker_pose_l;
        joint_state.msg_js.position[8] = rocker_bogie_encoders.msg.rocker_pose_r;
        joint_state.msg_js.position[9] = rocker_bogie_encoders.msg.bogie_pose_l;
        joint_state.msg_js.position[10] = rocker_bogie_encoders.msg.bogie_pose_r;
        joint_state.msg_js.position[11] = 0;
        joint_state.msg_js.position[12] = 0;
        joint_state.msg_js.position[13] = 0;
        joint_state.msg_js.position[14] = 0;
        joint_state.msg_js.position[15] = 0;
        joint_state.msg_js.position[16] = 0;

        joint_state.msg_js.position[0] = LINK_0.IMP2RAD(arm_encoders.msg.link_pose_0);
        joint_state.msg_js.position[1] = LINK_1.IMP2RAD(arm_encoders.msg.link_pose_1);
        joint_state.msg_js.position[2] = LINK_2.IMP2RAD(arm_encoders.msg.link_pose_2);
        joint_state.msg_js.position[3] = LINK_3.IMP2RAD(arm_encoders.msg.link_pose_3);
        joint_state.msg_js.position[4] = LINK_4.IMP2RAD(arm_encoders.msg.link_pose_4);
        joint_state.msg_js.position[5] = GRIPPER.IMP2RAD(arm_encoders.msg.grip_pose);
        joint_state.msg_js.position[6] = GRIPPER.IMP2RAD(arm_encoders.msg.grip_pose);

        joint_state.msg_js.position[7] = ROCKER.IMP2RAD(rocker_bogie_encoders.msg.rocker_pose_l);
        joint_state.msg_js.position[8] = ROCKER.IMP2RAD(rocker_bogie_encoders.msg.rocker_pose_r);
        joint_state.msg_js.position[9] = BOGIE.IMP2RAD(rocker_bogie_encoders.msg.bogie_pose_l);
        joint_state.msg_js.position[10] = BOGIE.IMP2RAD(rocker_bogie_encoders.msg.bogie_pose_r);

        joint_state.msg_js.position[11] = WHEEL.IMP2RAD(wheels_encoders.msg.wheel_vel_fl);
        joint_state.msg_js.position[12] = WHEEL.IMP2RAD(wheels_encoders.msg.wheel_vel_fr);
        joint_state.msg_js.position[13] = WHEEL.IMP2RAD(wheels_encoders.msg.wheel_vel_ml);
        joint_state.msg_js.position[14] = WHEEL.IMP2RAD(wheels_encoders.msg.wheel_vel_mr);
        joint_state.msg_js.position[15] = WHEEL.IMP2RAD(wheels_encoders.msg.wheel_vel_bl);
        joint_state.msg_js.position[16] = WHEEL.IMP2RAD(wheels_encoders.msg.wheel_vel_br);

        joint_state.Publish();

        security_error.Publish(error_control.msg.data);

        loop_rate.sleep();
    }
    return 0;
}
