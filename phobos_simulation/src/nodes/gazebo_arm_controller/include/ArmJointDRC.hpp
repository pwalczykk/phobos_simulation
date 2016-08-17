#ifndef ARMJOINTDR_HPP_
#define ARMJOINTDR_HPP_

#include "dynamic_reconfigure/server.h"
#include "phobos_simulation/ArmJointDRCConfig.h"

#include "ArmJointsController.hpp"

class ArmJointDRC : public ArmJointsController{
    ros::NodeHandle nh;
    dynamic_reconfigure::Server<phobos_simulation::ArmJointDRCConfig> server;
    dynamic_reconfigure::Server<phobos_simulation::ArmJointDRCConfig>::CallbackType f;

public:
    ArmJointDRC() : ArmJointsController()
    {
        f = boost::bind(&ArmJointDRC::DRCallback, this,_1, _2);
        server.setCallback(f);
    };
    ~ArmJointDRC(){};

    void DRCallback(phobos_simulation::ArmJointDRCConfig &config, uint32_t level)
    {
        ArmJointsController::pid[0].Set(config.link_0_Kp, config.link_0_Ki, config.link_0_Kd);
        ArmJointsController::pid[1].Set(config.link_1_Kp, config.link_1_Ki, config.link_1_Kd);
        ArmJointsController::pid[2].Set(config.link_2_Kp, config.link_2_Ki, config.link_2_Kd);
        ArmJointsController::pid[3].Set(config.link_3_Kp, config.link_3_Ki, config.link_3_Kd);
        ArmJointsController::pid[4].Set(config.link_4_Kp, config.link_4_Ki, config.link_4_Kd);
        ArmJointsController::pid[5].Set(config.finger_Kp, config.finger_Ki, config.finger_Kd);
        ArmJointsController::pid[6].Set(config.finger_Kp, config.finger_Ki, config.finger_Kd);
    }
};

#endif
