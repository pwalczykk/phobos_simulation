#ifndef GZTF_HPP_
#define GZTF_HPP_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "GzIMU.hpp"

class GzTF{
    ros::NodeHandle nh;

    GzIMU* imu;

    tf::TransformBroadcaster odom_broadcaster;

    geometry_msgs::TransformStamped odom_trans;
public:
     GzTF(GzIMU* imu){
         this->imu = imu;

         this->odom_trans.header.frame_id = "odom";
         this->odom_trans.child_frame_id = "frame_imu";
     }

     void Update(){
         this->odom_trans.header.seq = imu->msg_imu.header.seq;
         this->odom_trans.header.stamp = imu->msg_imu.header.stamp;

         //this->odom_trans.child_frame_id = "world";

         this->odom_trans.transform.rotation = imu->msg_imu.orientation;

         odom_broadcaster.sendTransform(odom_trans);
     }
};
#endif
