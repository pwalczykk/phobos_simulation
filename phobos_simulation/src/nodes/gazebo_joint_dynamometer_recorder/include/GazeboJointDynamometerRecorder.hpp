#ifndef JOINT_DYNAMOMETER_RECORDER_HPP
#define JOINT_DYNAMOMETER_RECORDER_HPP

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include <fstream>

class GazeboJointDynamometerRecorder{
    ros::NodeHandle nh;
    ros::Subscriber sub;

    std::fstream file;

    std::string file_name;

public:
    GazeboJointDynamometerRecorder(std::string file_name, std::string topic){
        this->sub = nh.subscribe(topic, 100, &GazeboJointDynamometerRecorder::UpdateWrench, this);
        this->file_name = file_name;
    }
    ~GazeboJointDynamometerRecorder(){
    };

    void UpdateWrench(const geometry_msgs::Wrench msg){
        this->file.open(file_name, std::ios_base::out | std::ios_base::app);
        if(!this->file.is_open()){
            ROS_WARN("File not opened -> Not saving!");
            return;
        }

        file << msg.force.x << " " << msg.force.y << " " << msg.force.z << " "
        << msg.torque.x << " " << msg.torque.y << " " << msg.torque.z << std::endl;

        this->file.close();

        return;
    }
};


#endif
