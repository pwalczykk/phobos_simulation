#include "../include/GazeboJointDynamometerRecorder.hpp"

int main(int argc, char** argv){

    if(argc != 3 && argc != 5){
        ROS_ERROR("Required arguments: FILE_NAME TOPIC_NAME");
        exit(-1);
    }

    // std::string file_name = argv[1];
    // std::string topic_name = argv[2];

    ros::init(argc, argv, "joint_dynamometer_recorder");

    ROS_WARN_STREAM(argv[argc-2] << " " << argv[argc-1]);

    GazeboJointDynamometerRecorder recorder(argv[argc-2], argv[argc-1]);

    ros::spin();

    return 0;
}
