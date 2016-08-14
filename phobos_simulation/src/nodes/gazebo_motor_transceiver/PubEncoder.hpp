#ifndef PUBECODER_HPP_
#define PUBECODER_HPP_

template <typename ros_Message>
class PubEncoder{
    ros::NodeHandle *nh;
    ros::Publisher pub;
public:
    ros_Message msg;

public:
    PubEncoder(std::string topic, ros::NodeHandle *nh){
        this->nh = nh;
        this->pub = nh->advertise<ros_Message>(topic, 100);
    }

    void Publish(){
        this->pub.publish(msg);
    }
};

#endif
