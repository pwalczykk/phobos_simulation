#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"

#include "std_msgs/Float64.h"

#include <string>
#include <sstream>
#include <thread>

#include "ControllerPID.hh"

namespace gazebo
{
class DifferentialPID : public ModelPlugin, public ControllerPID
{
private:
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Publisher pub_error;
    ros::Publisher pub_force;

    ros::Subscriber sub_Kp;
    ros::Subscriber sub_Ki;
    ros::Subscriber sub_Kd;

    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;
    event::ConnectionPtr updateConnection;

protected:
    physics::ModelPtr __model;
    physics::JointPtr joint_left;
    physics::JointPtr joint_right;

    double controller_rate;
    double simulation_rate;
    double number;


    std_msgs::Float64 msg_f64;

protected:



    double angle_left;
    double angle_right;

    double error;
    double force;


public: // Core functions
    DifferentialPID();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

public: // Control functions
    void OnUpdate(const common::UpdateInfo &info);
    void OnPerioid();
    void OnRosMsg_Kp(const std_msgs::Float64ConstPtr &_msg);
    void OnRosMsg_Ki(const std_msgs::Float64ConstPtr &_msg);
    void OnRosMsg_Kd(const std_msgs::Float64ConstPtr &_msg);

private: // System functions
    void Load_SimLoop();
    void Load_NodeROS();
    void Load_Subscrieber(std::string topic,
                          ros::Subscriber& sub,
                          void (DifferentialPID::*function)(const std_msgs::Float64ConstPtr&));

    void Load_Publisher(std::string topic, ros::Publisher& pub);

    void QueueThread();
};
GZ_REGISTER_MODEL_PLUGIN(DifferentialPID)
}
