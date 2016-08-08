#ifndef SUB_VEL_HPP_
#define SUB_VEL_HPP_

template <typename ros_Message>
class SubVel{
    ros::NodeHandle *nh;
    ros::Subscriber sub;
public:
    ros_Message msg;
public:
    SubVel(std::string topic, ros::NodeHandle *nh){
        this->nh = nh;
        this->sub = nh->subscribe(topic, 100, &SubVel::MsgInterrupt, this);
    }
    ~SubVel(){}

    void MsgInterrupt(const ros_Message msg){
        this->msg = msg;
    }
};
#endif
