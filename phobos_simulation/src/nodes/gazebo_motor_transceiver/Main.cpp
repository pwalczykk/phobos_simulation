#include <ros/ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

#include <phobos_shared/ArmVel16.h>
#include <phobos_shared/WheelsVel16.h>
#include <phobos_shared/EncodersArm.h>
#include <phobos_shared/EncodersWheels.h>
#include <phobos_shared/EncodersRockerBogie.h>

#include "SubVel.hpp"
#include "PubCtrl.hpp"
#include "SubEncoder.hpp"
#include "PubEncoder.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "gazebo_motor_transceiver");
    ros::NodeHandle nh;

    SubVel <phobos_shared::ArmVel16>sub_ctrl_arm_vel("/rover/control/arm_vel", &nh);
    SubVel <phobos_shared::WheelsVel16>sub_ctrl_wheels_vel("/rover/control/wheels_vel", &nh);

    PubEncoder <phobos_shared::EncodersArm>pub_encoder_arm("/rover/encoders/arm_absolute", &nh);
    PubEncoder <phobos_shared::EncodersWheels>pub_encoder_wheels("/rover/encoders/wheels_relative", &nh);
    PubEncoder <phobos_shared::EncodersRockerBogie>pub_encoder_rocker_bogie("/rover/encoders/rocker_bogie_absolute", &nh);


    PubCtrl <std_msgs::Float64>pub_ctrl_wheel_fl("/simulation/controller/vel/wheel_fl", &nh, -10, 10);
    PubCtrl <std_msgs::Float64>pub_ctrl_wheel_fr("/simulation/controller/vel/wheel_fr", &nh, -10, 10);
    PubCtrl <std_msgs::Float64>pub_ctrl_wheel_ml("/simulation/controller/vel/wheel_ml", &nh, -10, 10);
    PubCtrl <std_msgs::Float64>pub_ctrl_wheel_mr("/simulation/controller/vel/wheel_mr", &nh, -10, 10);
    PubCtrl <std_msgs::Float64>pub_ctrl_wheel_bl("/simulation/controller/vel/wheel_bl", &nh, -10, 10);
    PubCtrl <std_msgs::Float64>pub_ctrl_wheel_br("/simulation/controller/vel/wheel_br", &nh, -10, 10);

    PubCtrl <std_msgs::Float64>pub_ctrl_link_0("/simulation/controller/vel/link_0", &nh, -1, 1);
    PubCtrl <std_msgs::Float64>pub_ctrl_link_1("/simulation/controller/vel/link_1", &nh, -1, 1);
    PubCtrl <std_msgs::Float64>pub_ctrl_link_2("/simulation/controller/vel/link_2", &nh, -1, 1);
    PubCtrl <std_msgs::Float64>pub_ctrl_link_3("/simulation/controller/vel/link_3", &nh, -1, 1);
    PubCtrl <std_msgs::Float64>pub_ctrl_link_4("/simulation/controller/vel/link_4", &nh, -1, 1);

    PubCtrl <std_msgs::Float64>pub_ctrl_finger_l("/simulation/controller/vel/finger_l", &nh, -1, 1);
    PubCtrl <std_msgs::Float64>pub_ctrl_finger_r("/simulation/controller/vel/finger_r", &nh, -1, 1);


    SubEncoder <std_msgs::Int16>sub_encoder_wheel_fl("/simulation/encoder/rel/wheel_fl", &nh);
    SubEncoder <std_msgs::Int16>sub_encoder_wheel_fr("/simulation/encoder/rel/wheel_fr", &nh);
    SubEncoder <std_msgs::Int16>sub_encoder_wheel_ml("/simulation/encoder/rel/wheel_ml", &nh);
    SubEncoder <std_msgs::Int16>sub_encoder_wheel_mr("/simulation/encoder/rel/wheel_mr", &nh);
    SubEncoder <std_msgs::Int16>sub_encoder_wheel_bl("/simulation/encoder/rel/wheel_bl", &nh);
    SubEncoder <std_msgs::Int16>sub_encoder_wheel_br("/simulation/encoder/rel/wheel_br", &nh);

    SubEncoder <std_msgs::Int16>sub_encoder_link_0("/simulation/encoder/abs/link_0", &nh);
    SubEncoder <std_msgs::Int16>sub_encoder_link_1("/simulation/encoder/abs/link_1", &nh);
    SubEncoder <std_msgs::Int16>sub_encoder_link_2("/simulation/encoder/abs/link_2", &nh);
    SubEncoder <std_msgs::Int16>sub_encoder_link_3("/simulation/encoder/abs/link_3", &nh);
    SubEncoder <std_msgs::Int16>sub_encoder_link_4("/simulation/encoder/abs/link_4", &nh);

    SubEncoder <std_msgs::Int16>sub_encoder_finger_l("/simulation/encoder/abs/finger_l", &nh);
    SubEncoder <std_msgs::Int16>sub_encoder_finger_r("/simulation/encoder/abs/finger_r", &nh);

    SubEncoder <std_msgs::Int16>sub_encoder_rocker_l("/simulation/encoder/abs/rocker_l_bearing", &nh);
    SubEncoder <std_msgs::Int16>sub_encoder_rocker_r("/simulation/encoder/abs/rocker_r_bearing", &nh);
    SubEncoder <std_msgs::Int16>sub_encoder_bogie_l("/simulation/encoder/abs/bogie_l_bearing", &nh);
    SubEncoder <std_msgs::Int16>sub_encoder_bogie_r("/simulation/encoder/abs/bogie_r_bearing", &nh);

    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();

        pub_ctrl_wheel_fl.MapAndPublish(sub_ctrl_wheels_vel.msg.wheels_left);
        pub_ctrl_wheel_ml.MapAndPublish(sub_ctrl_wheels_vel.msg.wheels_left);
        pub_ctrl_wheel_bl.MapAndPublish(sub_ctrl_wheels_vel.msg.wheels_left);
        pub_ctrl_wheel_fr.MapAndPublish(sub_ctrl_wheels_vel.msg.wheels_right);
        pub_ctrl_wheel_mr.MapAndPublish(sub_ctrl_wheels_vel.msg.wheels_right);
        pub_ctrl_wheel_br.MapAndPublish(sub_ctrl_wheels_vel.msg.wheels_right);

        pub_ctrl_link_0.MapAndPublish(sub_ctrl_arm_vel.msg.link_0);
        pub_ctrl_link_1.MapAndPublish(sub_ctrl_arm_vel.msg.link_1);
        pub_ctrl_link_2.MapAndPublish(sub_ctrl_arm_vel.msg.link_2);
        pub_ctrl_link_3.MapAndPublish(sub_ctrl_arm_vel.msg.link_3);
        pub_ctrl_link_4.MapAndPublish(sub_ctrl_arm_vel.msg.link_4);

        pub_ctrl_finger_l.MapAndPublish(sub_ctrl_arm_vel.msg.grip_force);
        pub_ctrl_finger_r.MapAndPublish(sub_ctrl_arm_vel.msg.grip_force);

            pub_encoder_wheels.msg.wheel_vel_fl = sub_encoder_wheel_fl.msg.data;
            pub_encoder_wheels.msg.wheel_vel_fr = sub_encoder_wheel_fr.msg.data;
            pub_encoder_wheels.msg.wheel_vel_ml = sub_encoder_wheel_ml.msg.data;
            pub_encoder_wheels.msg.wheel_vel_mr = sub_encoder_wheel_mr.msg.data;
            pub_encoder_wheels.msg.wheel_vel_bl = sub_encoder_wheel_bl.msg.data;
            pub_encoder_wheels.msg.wheel_vel_br = sub_encoder_wheel_br.msg.data;
        pub_encoder_wheels.Publish();

            pub_encoder_arm.msg.link_pose_0 = sub_encoder_link_0.msg.data;
            pub_encoder_arm.msg.link_pose_1 = sub_encoder_link_1.msg.data;
            pub_encoder_arm.msg.link_pose_2 = sub_encoder_link_2.msg.data;
            pub_encoder_arm.msg.link_pose_3 = sub_encoder_link_3.msg.data;
            pub_encoder_arm.msg.link_pose_4 = sub_encoder_link_4.msg.data;
            pub_encoder_arm.msg.grip_pose = sub_encoder_finger_l.msg.data;

        pub_encoder_arm.Publish();

            pub_encoder_rocker_bogie.msg.rocker_pose_l = sub_encoder_rocker_l.msg.data;
            pub_encoder_rocker_bogie.msg.rocker_pose_r = sub_encoder_rocker_r.msg.data;
            pub_encoder_rocker_bogie.msg.bogie_pose_l = sub_encoder_bogie_l.msg.data;
            pub_encoder_rocker_bogie.msg.bogie_pose_r = sub_encoder_bogie_r.msg.data;
        pub_encoder_rocker_bogie.Publish();

        loop_rate.sleep();
    }

    return 0;
}
