#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <phobos_shared/ArmVel16.h>
#include <phobos_shared/WheelsVel16.h>

#include "SubVel.hpp"
#include "PubCtrl.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "gazebo_spi_motor_transmiter");
    ros::NodeHandle nh;

    SubVel <phobos_shared::ArmVel16>sub_arm_vel("/rover/control/arm_vel", &nh);
    SubVel <phobos_shared::WheelsVel16>sub_wheels_vel("/rover/control/wheels_vel", &nh);

    PubCtrl <std_msgs::Float64>pub_wheel_fl("/simulation/controller/joint/wheel_fl", &nh, -10, 10);
    PubCtrl <std_msgs::Float64>pub_wheel_fr("/simulation/controller/joint/wheel_fr", &nh, -10, 10);
    PubCtrl <std_msgs::Float64>pub_wheel_ml("/simulation/controller/joint/wheel_ml", &nh, -10, 10);
    PubCtrl <std_msgs::Float64>pub_wheel_mr("/simulation/controller/joint/wheel_mr", &nh, -10, 10);
    PubCtrl <std_msgs::Float64>pub_wheel_bl("/simulation/controller/joint/wheel_bl", &nh, -10, 10);
    PubCtrl <std_msgs::Float64>pub_wheel_br("/simulation/controller/joint/wheel_br", &nh, -10, 10);

    PubCtrl <std_msgs::Float64>pub_link_0("/simulation/controller/joint/link_0", &nh, -1, 1);
    PubCtrl <std_msgs::Float64>pub_link_1("/simulation/controller/joint/link_1", &nh, -1, 1);
    PubCtrl <std_msgs::Float64>pub_link_2("/simulation/controller/joint/link_2", &nh, -1, 1);
    PubCtrl <std_msgs::Float64>pub_link_3("/simulation/controller/joint/link_3", &nh, -1, 1);
    PubCtrl <std_msgs::Float64>pub_link_4("/simulation/controller/joint/link_4", &nh, -1, 1);

    PubCtrl <std_msgs::Float64>pub_finger_l("/simulation/controller/joint/finger_l", &nh, -1, 1);
    PubCtrl <std_msgs::Float64>pub_finger_r("/simulation/controller/joint/finger_r", &nh, -1, 1);

    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();

        pub_wheel_fl.MapAndPublish(sub_wheels_vel.msg.wheels_left);
        pub_wheel_ml.MapAndPublish(sub_wheels_vel.msg.wheels_left);
        pub_wheel_bl.MapAndPublish(sub_wheels_vel.msg.wheels_left);
        pub_wheel_fr.MapAndPublish(sub_wheels_vel.msg.wheels_right);
        pub_wheel_mr.MapAndPublish(sub_wheels_vel.msg.wheels_right);
        pub_wheel_br.MapAndPublish(sub_wheels_vel.msg.wheels_right);

        pub_link_0.MapAndPublish(sub_arm_vel.msg.link_0);
        pub_link_1.MapAndPublish(sub_arm_vel.msg.link_1);
        pub_link_2.MapAndPublish(sub_arm_vel.msg.link_2);
        pub_link_3.MapAndPublish(sub_arm_vel.msg.link_3);
        pub_link_4.MapAndPublish(sub_arm_vel.msg.link_4);

        pub_finger_l.MapAndPublish(sub_arm_vel.msg.grip_force);
        pub_finger_r.MapAndPublish(sub_arm_vel.msg.grip_force);

        loop_rate.sleep();
    }

    return 0;
}
