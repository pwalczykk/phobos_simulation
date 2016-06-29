#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>
#include <thread>
#include <string>
#include <sstream>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Wrench.h"

namespace gazebo
{
    class JointDynamometer : public ModelPlugin
    {
        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::Subscriber rosSub;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        event::ConnectionPtr updateConnection;

        physics::ModelPtr model;
        physics::JointPtr joint;

        ros::Publisher pub;

        geometry_msgs::Wrench msg;
        physics::JointWrench wrench;

    public:
        JointDynamometer();

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void Load_Publisher(std::string topic, ros::Publisher& pub);
        void Load_SimLoop();
        void Load_NodeROS();

        void OnUpdate(const common::UpdateInfo &info);

    private:
        void QueueThread();
    };
    GZ_REGISTER_MODEL_PLUGIN(JointDynamometer)
}
