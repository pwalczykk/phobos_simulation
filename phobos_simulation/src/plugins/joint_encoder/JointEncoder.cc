#include "JointEncoder.hh"

namespace gazebo
{

JointEncoder::JointEncoder() : ModelPlugin()
{
    this->counter = 0;
    this->period = 100;
}

void JointEncoder::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    std::string joint_name, topic_name;
    joint_name = _sdf->GetElement("joint")->GetValue()->GetAsString();
    topic_name = _sdf->GetElement("topic")->GetValue()->GetAsString();

    _sdf->GetElement("upper")->GetValue()->Get(this->highest_position);
    _sdf->GetElement("lower")->GetValue()->Get(this->lowest_position);

    _sdf->GetElement("absolute")->GetValue()->Get(this->absolute);
    _sdf->GetElement("resolution")->GetValue()->Get(this->resolution);


    this->current_position = lowest_position;
    this->prev_position = lowest_position;
    this->impulse_per_radian = resolution / 3.1416;

    this->model = _parent;
    this->joint = this->model->GetJoint(joint_name);

    this->Load_SimLoop();
    this->Load_NodeROS();

    this->Load_Publisher(topic_name, this->pub);

    printf("JointEncoder plugin launched succesfully\n");
}

void JointEncoder::OnUpdate(const common::UpdateInfo &info)
{
    counter++;
    if(counter == period){
        counter = 0;

        if(this->absolute){
            current_position = this->joint->GetAngle(0).Radian();
            encoder_value = (int16_t) ((current_position - lowest_position) * impulse_per_radian);
        }
        else{
            prev_position = current_position;
            current_position = this->joint->GetAngle(0).Radian();
            encoder_value = (int16_t) ((current_position - prev_position) * impulse_per_radian);
        }

        msg.data = encoder_value;

        this->pub.publish(msg);

    }
}

void JointEncoder::Load_Publisher(std::string topic, ros::Publisher& pub)
{
    pub = this->rosNode->advertise<std_msgs::Int16>(topic, 100);
}

void JointEncoder::Load_SimLoop()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&JointEncoder::OnUpdate, this, _1));
    printf("SimLoop plugin launched succesfully\n");
}

void JointEncoder::Load_NodeROS()
{
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    this->rosQueueThread =
    std::thread(std::bind(&JointEncoder::QueueThread, this));

    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
    }
}

void JointEncoder::QueueThread()
{
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}
}
