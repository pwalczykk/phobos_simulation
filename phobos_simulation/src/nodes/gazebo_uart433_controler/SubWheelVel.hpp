#ifndef SUBWHEELVEL_HPP_
#define SUBWHEELVEL_HPP_

#include <ros/ros.h>
#include <phobos_shared/WheelsVel16.h>

class SubWheelVel{
    ros::NodeHandle *nh;
    ros::Subscriber sub;

    PubGazeboJointVel *pub;
public:
    SubWheelVel(std::string topic, ros::NodeHandle *nh, PubGazeboJointVel *pub){
        this->nh = nh;
        this->pub = pub;
        this->sub = nh->subscribe(topic, 100, &SubWheelVel::MsgInterrupt, this);
    }
    ~SubWheelVel(){}

    void MsgInterrupt(const phobos_shared::WheelsVel16 msg){
        pub->UpdateVel(7, msg.wheels_left);
        pub->UpdateVel(9, msg.wheels_left);
        pub->UpdateVel(11, msg.wheels_left);

        pub->UpdateVel(8, msg.wheels_right);
        pub->UpdateVel(10, msg.wheels_right);
        pub->UpdateVel(12, msg.wheels_right);
    }
};

#endif
