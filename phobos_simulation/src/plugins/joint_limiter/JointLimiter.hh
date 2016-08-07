#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>
#include <thread>
#include <string>
#include <sstream>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"

namespace gazebo
{
    class JointLimiter : public ModelPlugin
    {
        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::Subscriber rosSub;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;

        physics::ModelPtr model;
        physics::JointPtr joint;


        math::Angle current_position;
        math::Angle highest_position;
        math::Angle lowest_position;

    public:
        JointLimiter();

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        void OnRosMsg(const std_msgs::Float64ConstPtr &_msg);

    private:
        void QueueThread();
    };
    GZ_REGISTER_MODEL_PLUGIN(JointLimiter)
}
