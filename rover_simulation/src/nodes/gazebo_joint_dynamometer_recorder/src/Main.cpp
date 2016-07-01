#include "../include/GazeboJointDynamometerRecorder.hpp"

int main(int argc, char** argv){

    if(argc != 3){
        ROS_ERROR("Required arguments: FILE_NAME TOPIC_NAME");
        exit(-1);
    }

    // std::string file_name = argv[1];
    // std::string topic_name = argv[2];

    ros::init(argc, argv, "joint_dynamometer_recorder");

    GazeboJointDynamometerRecorder recorder(argv[1], argv[2]);

    ros::spin();

    return 0;
}
