#ifndef SUBENCODER_HPP_
#define SUBENCODER_HPP_

template <typename ros_Message>
class SubEncoder{
    ros::NodeHandle *nh;
    ros::Subscriber sub;
public:
    ros_Message msg;
public:
    SubEncoder(std::string topic, ros::NodeHandle *nh){
        this->nh = nh;
        this->sub = nh->subscribe(topic, 100, &SubEncoder::MsgInterrupt, this);
    }
    ~SubEncoder(){}

    void MsgInterrupt(const ros_Message msg){
        this->msg = msg;
    }
};
#endif
