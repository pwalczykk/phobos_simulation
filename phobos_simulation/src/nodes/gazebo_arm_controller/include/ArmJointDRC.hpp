#ifndef ARMJOINTDR_HPP_
#define ARMJOINTDR_HPP_

#include "dynamic_reconfigure/server.h"
#include "rover_simulation/ArmJointDRCConfig.h"

#include "ArmJointsController.hpp"

class ArmJointDRC : public ArmJointsController{
    ros::NodeHandle nh;
    dynamic_reconfigure::Server<rover_simulation::ArmJointDRCConfig> server;
    dynamic_reconfigure::Server<rover_simulation::ArmJointDRCConfig>::CallbackType f;

public:
    ArmJointDRC() : ArmJointsController()
    {
        f = boost::bind(&ArmJointDRC::DRCallback, this,_1, _2);
        server.setCallback(f);
    };
    ~ArmJointDRC(){};

    void DRCallback(rover_simulation::ArmJointDRCConfig &config, uint32_t level)
    {
        ArmJointsController::pid[0].Set(config.link0_Kp, config.link0_Ki, config.link0_Kd);
        ArmJointsController::pid[1].Set(config.link1_Kp, config.link1_Ki, config.link1_Kd);
        ArmJointsController::pid[2].Set(config.link2_Kp, config.link2_Ki, config.link2_Kd);
        ArmJointsController::pid[3].Set(config.link3_Kp, config.link3_Ki, config.link3_Kd);
        ArmJointsController::pid[4].Set(config.link4_Kp, config.link4_Ki, config.link4_Kd);
        ArmJointsController::pid[5].Set(config.finger_Kp, config.finger_Ki, config.finger_Kd);
        ArmJointsController::pid[6].Set(config.finger_Kp, config.finger_Ki, config.finger_Kd);
    }
};

#endif
