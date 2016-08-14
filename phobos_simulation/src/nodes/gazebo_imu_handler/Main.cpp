#include <ros/ros.h>
#include "GzLinkState.hpp"
#include "GzIMU.hpp"
#include "GzTF.hpp"

#define RATE 10

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "gazebo_imu_handler");
    ros::NodeHandle nh;
    ros::Rate loop_rate(RATE);

    GzIMU __imu("base_link", "/rover/sensors/imu");
    GzTF __tf(&__imu);

    while(ros::ok()){
        __imu.Update();
        __tf.Update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
