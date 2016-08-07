#ifndef PARAMS_SERVER_HPP_
#define PARAMS_SERVER_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "dynamic_reconfigure/server.h"
#include "phobos_simulation/ControllerServerConfig.h"
#include <string>
#include <sstream>

class ControllerServer
{
    double Kp, Ki, Kd;

    std_msgs::Float64 msg;

    ros::NodeHandle nh;

    ros::Publisher pub_Ki;
    ros::Publisher pub_Kd;
    ros::Publisher pub_Kp;

    dynamic_reconfigure::Server<phobos_simulation::ControllerServerConfig> server;
    dynamic_reconfigure::Server<phobos_simulation::ControllerServerConfig>::CallbackType f;

public:
    ControllerServer(std::string topic, double Kp, double Ki, double Kd);
    void SetPID(double Kp, double Ki, double Kd);
    void DRCallback(phobos_simulation::ControllerServerConfig &config, uint32_t level);

};

#endif
