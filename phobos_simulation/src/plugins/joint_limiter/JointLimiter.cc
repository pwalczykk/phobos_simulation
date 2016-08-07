#include "JointLimiter.hh"

namespace gazebo
{

JointLimiter::JointLimiter() : ModelPlugin()
{
}

void JointLimiter::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    std::string joint_name, topic_name;

    // POBIERANIE ARGUMENTÓW WYWOŁANIA W URDF
    {   // TOPIKI
        joint_name = _sdf->GetElement("joint")->GetValue()->GetAsString();
        topic_name = _sdf->GetElement("topic")->GetValue()->GetAsString();

        std::stringstream ss;
        ss << topic_name << "/command";
        topic_name = ss.str();
    }
    {   // LIMITY
        double temp_highest_position, temp_lowest_position;
        _sdf->GetElement("upper")->GetValue()->Get(temp_highest_position);
        _sdf->GetElement("lower")->GetValue()->Get(temp_lowest_position);

        this->highest_position = math::Angle(temp_highest_position);
        this->lowest_position = math::Angle(temp_lowest_position);
    }

    // POBIERANIE WKAŹNIKÓW DO MODELU I JOINTA
    {
        this->model = _parent;

        this->joint = this->model->GetJoint(joint_name);

    }

    // SUBSKRYBOWANIE
    {
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
        ros::SubscribeOptions so =
          ros::SubscribeOptions::create<std_msgs::Float64>(
              topic_name,
              1,
              boost::bind(&JointLimiter::OnRosMsg, this, _1),
              ros::VoidPtr(),
              &this->rosQueue);

        this->rosSub = this->rosNode->subscribe(so);

        this->rosQueueThread =
          std::thread(std::bind(&JointLimiter::QueueThread, this));
    }

    // SPRAWDZANIE INICJALIZACJI
    {
        if (!ros::isInitialized())
        {
          int argc = 0;
          char **argv = NULL;
          ros::init(argc, argv, "gazebo_client",
              ros::init_options::NoSigintHandler);
        }
        ROS_INFO_STREAM( joint_name << " limiter plugin launched succesfully");
    }

}

void JointLimiter::OnRosMsg(const std_msgs::Float64ConstPtr &_msg)
{
    if(_msg->data == 0)
    {
        current_position = joint->GetAngle(0);
        joint->SetHighStop(0, *current_position);
        joint->SetLowStop(0, *current_position);
    }else{
        joint->SetHighStop(0, *highest_position);
        joint->SetLowStop(0, *lowest_position);
    }
}

void JointLimiter::QueueThread()
{
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}
}
