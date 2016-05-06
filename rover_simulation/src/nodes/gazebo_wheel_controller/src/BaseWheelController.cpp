#include "../include/BaseWheelController.hpp"

BaseWheelController::BaseWheelController()
{
    sub_vel = nh.subscribe("/rover/cmd_vel", 100, &BaseWheelController::SubCmdVel, this);

    pub_wheel[0] = nh.advertise<std_msgs::Float64>("rover/wheel_fl_velocity_controller/command", 100);
    pub_wheel[1] = nh.advertise<std_msgs::Float64>("rover/wheel_fr_velocity_controller/command", 100);
    pub_wheel[2] = nh.advertise<std_msgs::Float64>("rover/wheel_ml_velocity_controller/command", 100);
    pub_wheel[3] = nh.advertise<std_msgs::Float64>("rover/wheel_mr_velocity_controller/command", 100);
    pub_wheel[4] = nh.advertise<std_msgs::Float64>("rover/wheel_bl_velocity_controller/command", 100);
    pub_wheel[5] = nh.advertise<std_msgs::Float64>("rover/wheel_br_velocity_controller/command", 100);

    wheel_stop.data = 0;
    wheel_free.data = 0xffff;
}

BaseWheelController::~BaseWheelController()
{}

void BaseWheelController::SubCmdVel(const geometry_msgs::Twist cmd_vel)
{
    ROS_WARN("%f, %f", cmd_vel.linear.x, cmd_vel.angular.z);
    left_wheels_speed.data = cmd_vel.linear.x + cmd_vel.angular.z;
    left_wheels_speed_08.data = left_wheels_speed.data*0.8;

    right_wheels_speed.data = cmd_vel.linear.x - cmd_vel.angular.z;
    right_wheels_speed_08.data = right_wheels_speed.data*0.8;

// TODO Zastąpić rozmytym kontrolerem
    if(cmd_vel.angular.z < -0.1)
    {
        pub_wheel[0].publish(left_wheels_speed);
        pub_wheel[2].publish(left_wheels_speed_08);
        pub_wheel[4].publish(left_wheels_speed);

        pub_wheel[1].publish(right_wheels_speed);
        pub_wheel[3].publish(right_wheels_speed);
        pub_wheel[5].publish(right_wheels_speed_08);
    }else if(cmd_vel.angular.z > 0.1){
        pub_wheel[0].publish(left_wheels_speed);
        pub_wheel[2].publish(left_wheels_speed);
        pub_wheel[4].publish(left_wheels_speed_08);

        pub_wheel[1].publish(right_wheels_speed);
        pub_wheel[3].publish(right_wheels_speed_08);
        pub_wheel[5].publish(right_wheels_speed);
    }else{
        pub_wheel[0].publish(left_wheels_speed);
        pub_wheel[2].publish(left_wheels_speed);
        pub_wheel[4].publish(left_wheels_speed);

        pub_wheel[1].publish(right_wheels_speed);
        pub_wheel[3].publish(right_wheels_speed);
        pub_wheel[5].publish(right_wheels_speed);
    }
}


void BaseWheelController::PubWheelSpeed()
{
    pub_wheel[0].publish(left_wheels_speed);
    pub_wheel[2].publish(left_wheels_speed);
    pub_wheel[4].publish(left_wheels_speed);

    pub_wheel[1].publish(right_wheels_speed);
    pub_wheel[3].publish(right_wheels_speed);
    pub_wheel[5].publish(right_wheels_speed);
}
