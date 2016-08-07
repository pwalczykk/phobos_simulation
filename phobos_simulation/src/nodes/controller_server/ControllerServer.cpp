#include "ControllerServer.hpp"

ControllerServer::ControllerServer(std::string topic, double Kp, double Ki, double Kd)
{
    std::string topic_Kp, topic_Ki, topic_Kd;
    std::stringstream ss_Kp, ss_Ki, ss_Kd;

    ss_Kp << topic << "/Kp"; topic_Kp = ss_Kp.str();
    ss_Ki << topic << "/Ki"; topic_Ki = ss_Ki.str();
    ss_Kd << topic << "/Kd"; topic_Kd = ss_Kd.str();

    pub_Kp = nh.advertise<std_msgs::Float64>(topic_Kp, 1000);
    pub_Ki = nh.advertise<std_msgs::Float64>(topic_Ki, 1000);
    pub_Kd = nh.advertise<std_msgs::Float64>(topic_Kd, 1000);

    f = boost::bind(&ControllerServer::DRCallback, this,_1, _2);
    server.setCallback(f);

}

void ControllerServer::SetPID(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    msg.data=Kp;
    while(pub_Kp.getNumSubscribers() == 0){}
    pub_Kp.publish(msg);

    this->Ki = Ki;
    msg.data=Ki;
    while(pub_Kp.getNumSubscribers() == 0){}
    pub_Ki.publish(msg);

    this->Kd = Kd;
    msg.data=Kd;
    while(pub_Kp.getNumSubscribers() == 0){}
    pub_Kd.publish(msg);

    return;
}


void ControllerServer::DRCallback(phobos_simulation::ControllerServerConfig &config, uint32_t level)
{
    if(this->Kp != config.Kp){
        this->Kp = config.Kp;
        msg.data = this->Kp;
        pub_Kp.publish(msg);
        ROS_INFO("msg Kp: %f", this->Kp);
    }
    if(this->Ki != config.Ki){
        this->Ki = config.Ki;
        msg.data = this->Ki;
        pub_Ki.publish(msg);
        ROS_INFO("msg Ki: %f", this->Ki);

    }
    if(this->Kd != config.Kd){
        this->Kd = config.Kd;
        msg.data = this->Kd;
        pub_Kd.publish(msg);
        ROS_INFO("msg Kd: %f", this->Kd);

    }
}
