#ifndef GZTF_HPP_
#define GZTF_HPP_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "GzGPS.hpp"

class GzTF{
    ros::NodeHandle nh;

    GzGPS* gps;

    tf::TransformBroadcaster odom_broadcaster;

    geometry_msgs::TransformStamped odom_trans;
public:
     GzTF(GzGPS* gps){
         this->gps = gps;

         this->odom_trans.header = gps->msg_odom.header;
         this->odom_trans.child_frame_id = gps->msg_odom.child_frame_id;
         this->odom_trans.transform.rotation = gps->msg_odom.pose.pose.orientation;

     }

     void Update(){
         this->odom_trans.header.seq = gps->msg_odom.header.seq;
         this->odom_trans.header.stamp = gps->msg_odom.header.stamp;

         this->odom_trans.transform.translation.x = gps->msg_odom.pose.pose.position.x;
         this->odom_trans.transform.translation.y = gps->msg_odom.pose.pose.position.y;
         this->odom_trans.transform.translation.z = gps->msg_odom.pose.pose.position.z;

         this->odom_trans.transform.rotation = gps->msg_odom.pose.pose.orientation;

         odom_broadcaster.sendTransform(odom_trans);
     }
};
#endif
