#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>
#include <thread>
#include <string>
#include <sstream>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Int16.h"

namespace gazebo
{
    class JointEncoder : public ModelPlugin
    {
        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        event::ConnectionPtr updateConnection;

        physics::ModelPtr model;
        physics::JointPtr joint;

        ros::Publisher pub;

        std_msgs::Int16 msg;

        int counter;
        int period;

        double highest_position;
        double lowest_position;
        double current_position;
        double prev_position;

        bool absolute;
        double resolution;
        double impulse_per_radian;

        int16_t encoder_value;

    public:
        JointEncoder();

        void OnUpdate(const common::UpdateInfo &info);

    private:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        void Load_Publisher(std::string topic, ros::Publisher& pub);

        void Load_SimLoop();
        void Load_NodeROS();

        void QueueThread();
    };
    GZ_REGISTER_MODEL_PLUGIN(JointEncoder)
}
