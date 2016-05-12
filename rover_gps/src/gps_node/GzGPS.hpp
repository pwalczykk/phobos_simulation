#ifndef GZGPS_HPP_
#define GZGPS_HPP_

#include "GzLinkState.hpp"
#include "DRNoiseGen.hpp"
#include "nav_msgs/Odometry.h"

class GzTF;

class GzGPS : public GzLinkState, public DRNoiseGen{
    ros::NodeHandle nh;
    ros::Publisher pub_odom;

protected:
    nav_msgs::Odometry msg_odom;
    friend class GzTF;

public:
    GzGPS(std::string link_name, std::string topic_odom) : GzLinkState(link_name), DRNoiseGen() {
        pub_odom = nh.advertise<nav_msgs::Odometry>(topic_odom,100);

        msg_odom.header.seq = 0;
        msg_odom.header.stamp = ros::Time::now();
        msg_odom.header.frame_id = "odom";
        msg_odom.child_frame_id = "frame_gps";

        msg_odom.pose.pose.orientation.x = 0;   // Identity quaternion
        msg_odom.pose.pose.orientation.y = 0;   // Identity quaternion
        msg_odom.pose.pose.orientation.z = 0;   // Identity quaternion
        msg_odom.pose.pose.orientation.w = 1;   // Identity quaternion

        double cov_x = 1;
        double cov_y = 1;
        double cov_z = 1;

        msg_odom.pose.covariance = {cov_x,      0,      0,      0,      0,      0,
                                        0,  cov_y,      0,      0,      0,      0,
                                        0,      0,  cov_z,      0,      0,      0,
                                        0,      0,      0, 999999,      0,      0,
                                        0,      0,      0,      0, 999999,      0,
                                        0,      0,      0,      0,      0, 999999};



    }

    ~GzGPS(){
        //Nothing :)
    }

    void Update(){
        GzLinkState::Update();

        msg_odom.header.seq++;
        msg_odom.header.stamp = ros::Time::now();
        msg_odom.pose.pose.position = GzLinkState::msg_link_state.response.link_state.pose.position;

        DRNoiseGen::GenerateX(&msg_odom.pose.pose.position.x);
        DRNoiseGen::GenerateY(&msg_odom.pose.pose.position.y);
        DRNoiseGen::GenerateZ(&msg_odom.pose.pose.position.z);

        pub_odom.publish(msg_odom);
    }

};

#endif
