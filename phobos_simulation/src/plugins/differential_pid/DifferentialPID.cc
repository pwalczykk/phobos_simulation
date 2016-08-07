#include "DifferentialPID.hh"

namespace gazebo
{

/////////////////// CORE FUNCTIONS //////////////////////////////

DifferentialPID::DifferentialPID() : ModelPlugin() , ControllerPID()
{
    this->number = 0;
    this->force = 0;
}

void DifferentialPID::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    std::string joint_left_name = _sdf->GetElement("leftJoint")->GetValue()->GetAsString();
    std::string joint_right_name = _sdf->GetElement("rightJoint")->GetValue()->GetAsString();

    this->__model = _parent;

    this->joint_left = this->__model->GetJoint(joint_left_name);
    this->joint_right = this->__model->GetJoint(joint_right_name);

    DifferentialPID::Set(0,0,0);

    if (_sdf->HasElement("Kp"))
        _sdf->GetElement("Kp")->GetValue()->Get(ControllerPID::Kp);

    if (_sdf->HasElement("Ki"))
        _sdf->GetElement("Ki")->GetValue()->Get(ControllerPID::Ki);

    if (_sdf->HasElement("Kd"))
        _sdf->GetElement("Kd")->GetValue()->Get(ControllerPID::Kd);

    if (_sdf->HasElement("limit"))
        _sdf->GetElement("limit")->GetValue()->Get(ControllerPID::limit);

    std::string topic = _sdf->GetElement("topic")->GetValue()->GetAsString();

    std::string         topic_Kp, topic_Ki, topic_Kd;
    std::stringstream   ss_Kp,    ss_Ki,    ss_Kd;

    ss_Kp << topic << "/Kp"; topic_Kp = ss_Kp.str();
    ss_Ki << topic << "/Ki"; topic_Ki = ss_Ki.str();
    ss_Kd << topic << "/Kd"; topic_Kd = ss_Kd.str();

    std::string         topic_error, topic_force;
    std::stringstream   ss_error, ss_force;

    ss_error << topic << "/error"; topic_error = ss_error.str();

    ss_force<< topic << "/force"; topic_force = ss_force.str();


    this->Load_SimLoop();
    this->Load_NodeROS();

    this->Load_Publisher(topic_error, pub_error);
    this->Load_Publisher(topic_force, pub_force);

    this->Load_Subscrieber(topic_Kp, sub_Kp, &DifferentialPID::OnRosMsg_Kp);
    this->Load_Subscrieber(topic_Ki, sub_Ki, &DifferentialPID::OnRosMsg_Ki);
    this->Load_Subscrieber(topic_Kd, sub_Kd, &DifferentialPID::OnRosMsg_Kd);

    printf("DifferentialPID plugin launched succesfully\n");
}



/////////////////////// CONTROL FUNCTIONS ////////////////////////////

void DifferentialPID::OnUpdate(const common::UpdateInfo &info)
{
    this->OnPerioid();

    this->joint_left->SetForce(0,-1*this->force);
    this->joint_right->SetForce(0,-1*this->force);


}

void DifferentialPID::OnPerioid()
{
    number++; if(number > 4) { number = 0;
        this->angle_left = this->joint_left->GetAngle(0).Radian();
        this->angle_right = this->joint_right->GetAngle(0).Radian();

        this->error = angle_left + angle_right;

        this->force = ControllerPID::Controll(error);

        this->msg_f64.data = error;
        this->pub_error.publish(msg_f64);

        this->msg_f64.data = force;
        this->pub_force.publish(msg_f64);
    }
}

void DifferentialPID::OnRosMsg_Kp(const std_msgs::Float64ConstPtr &_msg)
{
    ControllerPID::Kp = _msg->data;
    DifferentialPID::ReadK();
}

void DifferentialPID::OnRosMsg_Ki(const std_msgs::Float64ConstPtr &_msg)
{
    ControllerPID::Ki = _msg->data;
    DifferentialPID::ReadK();
}

void DifferentialPID::OnRosMsg_Kd(const std_msgs::Float64ConstPtr &_msg)
{
    ControllerPID::Kd = _msg->data;
    DifferentialPID::ReadK();
}

///////// SYSTEM FUNCTIONS /////////////////////////

void DifferentialPID::QueueThread()
{
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}

void DifferentialPID::Load_SimLoop()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&DifferentialPID::OnUpdate, this, _1));
    printf("SimLoop plugin launched succesfully\n");
}

void DifferentialPID::Load_NodeROS()
{
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    this->rosQueueThread =
    std::thread(std::bind(&DifferentialPID::QueueThread, this));

    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
    }
}

void DifferentialPID::Load_Subscrieber(std::string topic,
                                        ros::Subscriber& sub,
                                        void (DifferentialPID::*function)(const std_msgs::Float64ConstPtr&))
{
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<std_msgs::Float64>(
          topic,
          1,
          boost::bind(function, this, _1),
          ros::VoidPtr(),
          &this->rosQueue);

    sub = this->rosNode->subscribe(so);
}


void DifferentialPID::Load_Publisher(std::string topic, ros::Publisher& pub)
{
    pub = this->rosNode->advertise<std_msgs::Float64>(topic, 100);
}

} //gazebo namespace
