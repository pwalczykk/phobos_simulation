#ifndef PUB_CTRL_HPP_
#define PUB_CTRL_HPP_

template <typename ros_Message>
class PubCtrl{
    ros::NodeHandle *nh;
    ros::Publisher pub;

    double alfa, beta; // Values used to project input Int16 <-32768 to 32767> message to output Float64 <min_val to max_val>
public:
    ros_Message msg;

public:
    PubCtrl(std::string topic, ros::NodeHandle *nh, double min_val, double max_val){
        this->nh = nh;
        this->pub = nh->advertise<ros_Message>(topic, 100);

        // alfa = (max_val - min_val) / 4294967296; // Int32
        alfa = (max_val - min_val) / 65536; // Int16
        beta = (min_val + max_val) / 2;
    }

    void MapAndPublish(int input){
        msg.data = alfa * (double) input + beta;
        this->pub.publish(msg);
    }
};

#endif
