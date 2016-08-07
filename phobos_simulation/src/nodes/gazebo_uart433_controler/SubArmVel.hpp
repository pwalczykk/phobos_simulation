#ifndef SUBARMVEL_HPP_
#define SUBARMVEL_HPP_

#include <ros/ros.h>
#include <phobos_shared/ArmVel16.h>

class SubArmVel{
    ros::NodeHandle *nh;
    ros::Subscriber sub;

    PubGazeboJointVel *pub;

public:
    SubArmVel(std::string topic, ros::NodeHandle *nh, PubGazeboJointVel *pub){
        this->nh = nh;
        this->pub = pub;
        this->sub = nh->subscribe(topic, 100, &SubArmVel::MsgInterrupt, this);
    }
    ~SubArmVel(){}

    void MsgInterrupt(const phobos_shared::ArmVel16 msg){
        pub->UpdateVel(0, msg.link_0);
        pub->UpdateVel(1, msg.link_1);
        pub->UpdateVel(2, msg.link_2);
        pub->UpdateVel(3, msg.link_3);
        pub->UpdateVel(4, msg.link_4);
    }
};

#endif
