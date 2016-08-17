#ifndef GZIMU_HPP_
#define GZIMU_HPP_

#include "GzLinkState.hpp"
#include "DRNoiseGen.hpp"

#include "sensor_msgs/Imu.h"

#define DERIVATIVE_BUFF 4

class GzTF;
class PubAccMagnitude;
class PubVertDevMagnitude;

class GzIMU : public GzLinkState, public DRNoiseGen{
    ros::NodeHandle nh;
    ros::Publisher pub_imu;

    double l_vel_x_buff[DERIVATIVE_BUFF];
    double l_vel_y_buff[DERIVATIVE_BUFF];
    double l_vel_z_buff[DERIVATIVE_BUFF];

protected:
    sensor_msgs::Imu msg_imu;
    friend class GzTF;
    friend class PubAccMagnitude;
    friend class PubVertDevMagnitude;


public:
    GzIMU(std::string link_name, std::string topic_imu) : GzLinkState(link_name), DRNoiseGen() {
        pub_imu = nh.advertise<sensor_msgs::Imu>(topic_imu,100);

        msg_imu.header.seq = 0;
        msg_imu.header.stamp = ros::Time::now();
        msg_imu.header.frame_id = "base_link";

        msg_imu.orientation_covariance = {1,   0,   0,
                                          0,   1,   0,
                                          0,   0,   1};

        msg_imu.angular_velocity_covariance = {1,   0,   0,
                                               0,   1,   0,
                                               0,   0,   1};

        msg_imu.linear_acceleration_covariance = {1,   0,   0,
                                                  0,   1,   0,
                                                  0,   0,   1};

    }

    ~GzIMU(){}

    double Derivative(double* d_buffor, double curr_val){
        memmove(d_buffor+1, d_buffor, (DERIVATIVE_BUFF-1)*sizeof(*d_buffor));
        d_buffor[0] = curr_val;

        //return (d_buffor[0] - d_buffor[1])/2;
        return (d_buffor[0] + 3*d_buffor[1] - 3*d_buffor[2] - d_buffor[3])/6;
    }

    void Update(){
        GzLinkState::Update();

        msg_imu.header.seq++;
        msg_imu.header.stamp = ros::Time::now();

        msg_imu.orientation = GzLinkState::msg_link_state.response.link_state.pose.orientation;

        msg_imu.angular_velocity = GzLinkState::msg_link_state.response.link_state.twist.angular;

        msg_imu.linear_acceleration.x = Derivative(l_vel_x_buff, GzLinkState::msg_link_state.response.link_state.twist.linear.x);
        msg_imu.linear_acceleration.y = Derivative(l_vel_y_buff, GzLinkState::msg_link_state.response.link_state.twist.linear.y);
        msg_imu.linear_acceleration.z = Derivative(l_vel_z_buff, GzLinkState::msg_link_state.response.link_state.twist.linear.z);

        DRNoiseGen::GenerateOrient(&msg_imu.orientation.x);
        DRNoiseGen::GenerateOrient(&msg_imu.orientation.y);
        DRNoiseGen::GenerateOrient(&msg_imu.orientation.z);
        DRNoiseGen::GenerateOrient(&msg_imu.orientation.w);

        DRNoiseGen::GenerateAngVel(&msg_imu.angular_velocity.x);
        DRNoiseGen::GenerateAngVel(&msg_imu.angular_velocity.y);
        DRNoiseGen::GenerateAngVel(&msg_imu.angular_velocity.z);

        DRNoiseGen::GenerateLinAcc(&msg_imu.linear_acceleration.x);
        DRNoiseGen::GenerateLinAcc(&msg_imu.linear_acceleration.y);
        DRNoiseGen::GenerateLinAcc(&msg_imu.linear_acceleration.z);

        pub_imu.publish(msg_imu);
    }
};

#endif
