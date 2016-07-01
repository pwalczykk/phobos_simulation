#include "JointDynamometer.hh"

namespace gazebo
{

JointDynamometer::JointDynamometer() : ModelPlugin()
{
    this->msg.force.x = 0;
    this->msg.force.y = 0;
    this->msg.force.z = 0;

    this->msg.torque.x = 0;
    this->msg.torque.y = 0;
    this->msg.torque.z = 0;

    this->period = 100;
    this->counter = 0;
}

void JointDynamometer::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    std::string joint_name = _sdf->GetElement("joint")->GetValue()->GetAsString();
    std::string topic_name = _sdf->GetElement("topic")->GetValue()->GetAsString();

    this->model = _parent;
    this->joint = this->model->GetJoint(joint_name);

    this->Load_SimLoop();
    this->Load_NodeROS();

    this->Load_Publisher(topic_name, pub);

    printf("JointDynamometer plugin launched succesfully\n");
}

///////////////////////////// CONTROL FUNCTIONS

void JointDynamometer::OnUpdate(const common::UpdateInfo &info)
{
    this->wrench = joint->GetForceTorque(0);    //Index not used currently
    this->msg.force.x += wrench.body1Force.x;
    this->msg.force.y += wrench.body1Force.y;
    this->msg.force.z += wrench.body1Force.z;

    this->msg.torque.x += wrench.body1Torque.x;
    this->msg.torque.y += wrench.body1Torque.y;
    this->msg.torque.z += wrench.body1Torque.z;

    counter++; if(counter > this->period) { counter = 0;

        this->msg.force.x /= this->period;
        this->msg.force.y /= this->period;
        this->msg.force.z /= this->period;

        this->msg.torque.x /= this->period;
        this->msg.torque.y /= this->period;
        this->msg.torque.z /= this->period;

        this->pub.publish(msg);
    }
}

///////////////////////////// SYSTEM FUNCTIONS

void JointDynamometer::Load_Publisher(std::string topic, ros::Publisher& pub)
{
    pub = this->rosNode->advertise<geometry_msgs::Wrench>(topic, 100);
}

// void JointDynamometer::Load_Subscrieber(std::string topic,
//                                         ros::Subscriber& sub,
//                                         void (JointDynamometer::*function)(const std_msgs::Float64ConstPtr&))
// {
//     ros::SubscribeOptions so =
//       ros::SubscribeOptions::create<std_msgs::Float64>(
//           topic,
//           1,
//           boost::bind(function, this, _1),
//           ros::VoidPtr(),
//           &this->rosQueue);
//
//     sub = this->rosNode->subscribe(so);
// }

void JointDynamometer::Load_SimLoop()
{
    ROS_ERROR("5");

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&JointDynamometer::OnUpdate, this, _1));
    printf("SimLoop plugin launched succesfully\n");
}

void JointDynamometer::Load_NodeROS()
{
    ROS_ERROR("6");

    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    this->rosQueueThread =
    std::thread(std::bind(&JointDynamometer::QueueThread, this));

    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
    }
}

void JointDynamometer::QueueThread()
{
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}
}
