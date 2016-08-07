#ifndef SUBARMVEL_HPP_
#define SUBARMVEL_HPP_

#include <ros/ros.h>
#include <rover_rpi_uart/ArmVel5.h>

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

    void MsgInterrupt(const rover_rpi_uart::ArmVel5 msg){
        pub->UpdateVel(0, msg.link0);
        pub->UpdateVel(1, msg.link1);
        pub->UpdateVel(2, msg.link2);
        pub->UpdateVel(3, msg.link3);
        pub->UpdateVel(4, msg.link4);
    }
};

#endif
