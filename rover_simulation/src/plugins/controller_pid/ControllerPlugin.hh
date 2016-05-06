#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

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
class ControllerPlugin : public ModelPlugin, public ControllerPID
{
private:
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Publisher pub_Vel;
    ros::Publisher pub_Pose;
    ros::Subscriber sub_Vel;

    ros::Subscriber sub_Kp;
    ros::Subscriber sub_Ki;
    ros::Subscriber sub_Kd;

    ros::Subscriber sub_reducer;

    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;
    event::ConnectionPtr updateConnection;

protected:
    physics::JointPtr __joint;
    physics::ModelPtr __model;

    double controller_rate;
    double simulation_rate;
    double angle;
    double number;

    double velocity_req;
    double velocity_curr;
    double pose_curr;

    double force;
//    double force_reductioin_factor;

    std_msgs::Float64 temp_msg;

public: // Core functions
    ControllerPlugin();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

public: // Control functions
    void OnUpdate(const common::UpdateInfo &info);
    void OnPerioid();
    void OnRosMsg_Vel(const std_msgs::Float64ConstPtr &_msg);
    void OnRosMsg_Kp(const std_msgs::Float64ConstPtr &_msg);
    void OnRosMsg_Ki(const std_msgs::Float64ConstPtr &_msg);
    void OnRosMsg_Kd(const std_msgs::Float64ConstPtr &_msg);

private: // System functions
    void Load_SimLoop();
    void Load_NodeROS();
    void Load_Subscrieber(std::string topic,
                          ros::Subscriber& sub,
                          void (ControllerPlugin::*function)(const std_msgs::Float64ConstPtr&));

    // void Load_Subscrieber_UInt8(std::string topic,
    //                             ros::Subscriber& sub,
    //                             void (ControllerPlugin::*function)(const std_msgs::UInt8ConstPtr&));


    void Load_Publisher(std::string topic, ros::Publisher& pub);

    void QueueThread();
};
GZ_REGISTER_MODEL_PLUGIN(ControllerPlugin)
}
