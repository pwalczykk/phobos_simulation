#include <ros/ros.h>
#include "GzLinkState.hpp"
#include "GzGPS.hpp"
#include "GzTF.hpp"

#define RATE 10

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "gps_sim_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(RATE);

    GzGPS __gps("base_link", "/rover/sensors/gps");
    GzTF __tf(&__gps);

    while(ros::ok()){
        __gps.Update();
        __tf.Update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
