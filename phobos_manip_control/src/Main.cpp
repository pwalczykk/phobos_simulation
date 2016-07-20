#include "ros/ros.h"
#include "../include/PublisherFloat64.hpp"
#include "../include/ReverseKinematicsCalculator.hpp"
#include "phobos_manip_control/PositionOrientation.h"

double pi = 3.14159265359;

PublisherFloat64 link0, link1, link2, link3, link4;
ReverseKinematicsCalculator cal;

void CalculateReverseKinematics(const phobos_manip_control::PositionOrientation::ConstPtr& msg)
{

  ROS_INFO("Data received: [X = %f, Y = %f, Z = %f, angle = %f]", msg->x, msg->y, msg->z, msg->angle);
  cal.SetPositionOrientation(msg->x, msg->y, msg->z, msg->angle);
  cal.CalculateReverseKinematics();
  ROS_INFO("Link0 = %f, Link1 = %f Link2 = %f Link3 = %f",cal.ReturnLink(0)*180/pi,cal.ReturnLink(1)*180/pi,cal.ReturnLink(2)*180/pi,cal.ReturnLink(3)*180/pi );

  link0.Publish(cal.ReturnLink(0));
  link1.Publish(cal.ReturnLink(1));
  link2.Publish(cal.ReturnLink(2));
  link3.Publish(cal.ReturnLink(3));

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "phobos_manip_control_node");
  ros::NodeHandle nh;

  cal.SetLengths(50,500,250,250);

  link0.Init("/rover/link0_position_controller/command", &nh);
  link1.Init("/rover/link1_position_controller/command", &nh);
  link2.Init("/rover/link2_position_controller/command", &nh);
  link3.Init("/rover/link3_position_controller/command", &nh);
  link4.Init("/rover/link4_position_controller/command", &nh);

  ros::Subscriber sub = nh.subscribe("phobos_position_orientation", 10, CalculateReverseKinematics);

  ros::spin();

  return 0;
}
