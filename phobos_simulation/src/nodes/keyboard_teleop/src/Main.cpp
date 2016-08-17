#include "../include/KeyListener.hpp"
#include "../include/PublisherFloat64.hpp"
#include "../include/PublisherTwist.hpp"
#include "../include/KeyDefinitions.hpp"

KeyListener key_listener;
PublisherFloat64 link_0, link_1, link_2, link_3, link_4, finger1, finger2, tower_yaw, tower_pitch;
PublisherFloat64 wheel_fl, wheel_fr, wheel_ml, wheel_mr, wheel_bl, wheel_br;
PublisherTwist cmd_vel;

double link_0_vel = 1.0;
double link_1_vel = 1.0;
double link_2_vel = 1.0;
double link_3_vel = 1.0;
double link_4_vel = 1.0;
double grip_vel = 1.0;
double tower_yaw_vel = 1.0;
double tower_pitch_vel = 1.0;

double rover_linear_1 = 4.0;
double rover_linear_2 = 3.0;
double rover_angular_1 = 6.0;
double rover_angular_2 = 4.5;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_teleop");
    ros::NodeHandle nh;

    key_listener.Init(&nh);

    link_0.Init("/rover/link_0_velocity_controller/command", &nh);
    link_1.Init("/rover/link_1_velocity_controller/command", &nh);
    link_2.Init("/rover/link_2_velocity_controller/command", &nh);
    link_3.Init("/rover/link_3_velocity_controller/command", &nh);
    link_4.Init("/rover/link_4_velocity_controller/command", &nh);
    finger1.Init("/rover/finger1_velocity_controller/command", &nh);
    finger2.Init("/rover/finger2_velocity_controller/command", &nh);

    tower_yaw.Init("/rover/tower_yaw_velocity_controller/command", &nh);
    tower_pitch.Init("/rover/tower_pitch_velocity_controller/command", &nh);

    // wheel_fl.Init("rover/wheel_fl_velocity_controller/command", &nh);
    // wheel_fr.Init("rover/wheel_fr_velocity_controller/command", &nh);
    // wheel_ml.Init("rover/wheel_ml_velocity_controller/command", &nh);
    // wheel_mr.Init("rover/wheel_mr_velocity_controller/command", &nh);
    // wheel_bl.Init("rover/wheel_bl_velocity_controller/command", &nh);
    // wheel_br.Init("rover/wheel_br_velocity_controller/command", &nh);

    cmd_vel.Init("/rover/cmd_vel", &nh);

    ros::spin();
}

void RoverVel(double linear_x, double angular_z)
{
    cmd_vel.Publish(linear_x, angular_z);

    // double left_wheels_speed = linear_x + angular_z;
    // double right_wheels_speed = linear_x - angular_z;
    //
    // wheel_fl.Publish(left_wheels_speed);
    // wheel_ml.Publish(left_wheels_speed);
    // wheel_bl.Publish(left_wheels_speed);
    //
    // wheel_fr.Publish(right_wheels_speed);
    // wheel_mr.Publish(right_wheels_speed);
    // wheel_br.Publish(right_wheels_speed);
}

void GripVel(double grip_vel)
{
    finger1.Publish(grip_vel);
    finger2.Publish(grip_vel);
}

void KeyDownCallback()
{
    switch(key_listener.ReturnKey())
    {
        case Q: link_0.Publish(link_0_vel); break;
        case A: link_0.Publish(-link_0_vel); break;

        case W: link_1.Publish(link_1_vel); break;
        case S: link_1.Publish(-link_1_vel); break;

        case E: link_2.Publish(link_2_vel); break;
        case D: link_2.Publish(-link_2_vel); break;

        case R: link_3.Publish(link_3_vel); break;
        case F: link_3.Publish(-link_3_vel); break;

        case T: link_4.Publish(link_4_vel); break;
        case G: link_4.Publish(-link_4_vel); break;

        case Y: GripVel(grip_vel); break;
        case H: GripVel(-grip_vel); break;

        case U: GripVel(grip_vel); break;
        case J: GripVel(0); break;

        case I: tower_yaw.Publish(tower_yaw_vel); break;
        case K: tower_yaw.Publish(-tower_yaw_vel); break;

        case O: tower_pitch.Publish(tower_pitch_vel); break;
        case L: tower_pitch.Publish(-tower_pitch_vel); break;

        case N1: RoverVel(-rover_linear_2,-rover_angular_2); break;
        case N2: RoverVel(-rover_linear_1,0); break;
        case N3: RoverVel(-rover_linear_2,rover_angular_2); break;
        case N4: RoverVel(0,-rover_angular_1); break;
        case N5: RoverVel(0,0); break;
        case N6: RoverVel(0,rover_angular_1); break;
        case N7: RoverVel(rover_linear_2,-rover_angular_2); break;
        case N8: RoverVel(rover_linear_1,0); break;
        case N9: RoverVel(rover_linear_2,rover_angular_2); break;
    }
}

void KeyUpCallback()
{
    switch(key_listener.ReturnKey())
    {
        case Q: case A: link_0.Publish(0); break;
        case W: case S: link_1.Publish(0); break;
        case E: case D: link_2.Publish(0); break;
        case R: case F: link_3.Publish(0); break;
        case T: case G: link_4.Publish(0); break;
        case Y: case H: GripVel(0); break;

        case I: case K: tower_yaw.Publish(0); break;
        case L: case O: tower_pitch.Publish(0); break;

        case N1: RoverVel(0,0); break;
        case N2: RoverVel(0,0); break;
        case N3: RoverVel(0,0); break;
        case N4: RoverVel(0,0); break;
        case N5: RoverVel(0,0); break;
        case N6: RoverVel(0,0); break;
        case N7: RoverVel(0,0); break;
        case N8: RoverVel(0,0); break;
        case N9: RoverVel(0,0); break;
    }
}
