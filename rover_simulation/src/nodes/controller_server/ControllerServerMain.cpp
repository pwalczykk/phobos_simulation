#include "ControllerServer.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ControllerServer");

    double Kp = atof(argv[argc-4]);
    double Ki = atof(argv[argc-3]);
    double Kd = atof(argv[argc-2]);

    ControllerServer server(argv[argc-1], Kp,Ki,Kd); // Jako argument przyjmuje topic dla nastawów regulatora
    ROS_ERROR("%f %f %f", Kp, Ki, Kd);
    server.SetPID(Kp, Ki, Kd);

    ros::spin();
    return 0;
}
