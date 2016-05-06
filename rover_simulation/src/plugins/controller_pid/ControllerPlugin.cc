#include "ControllerPlugin.hh"

namespace gazebo
{

/////////////////// CORE FUNCTIONS //////////////////////////////

ControllerPlugin::ControllerPlugin() : ModelPlugin() , ControllerPID()
{
    this->number = 0;
    this->force = 0;
}

void ControllerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    std::string joint_name = _sdf->GetElement("joint")->GetValue()->GetAsString();
    std::string topic = _sdf->GetElement("topic")->GetValue()->GetAsString();

    ControllerPlugin::Set(0,0,0);

    if (_sdf->HasElement("Kp"))
        _sdf->GetElement("Kp")->GetValue()->Get(ControllerPID::Kp);

    if (_sdf->HasElement("Ki"))
        _sdf->GetElement("Ki")->GetValue()->Get(ControllerPID::Ki);

    if (_sdf->HasElement("Kd"))
        _sdf->GetElement("Kd")->GetValue()->Get(ControllerPID::Kd);

    if (_sdf->HasElement("limit"))
        _sdf->GetElement("limit")->GetValue()->Get(ControllerPID::limit);


    std::string         topic_vel_curr, topic_vel_req, topic_pose_curr, topic_Kp, topic_Ki, topic_Kd;
    std::stringstream   ss_vel_curr,    ss_vel_req,    ss_pose_curr,    ss_Kp,    ss_Ki,    ss_Kd;

    ss_vel_req << topic << "/command"; topic_vel_req = ss_vel_req.str();
    ss_vel_curr << topic << "/vel_curr"; topic_vel_curr = ss_vel_curr.str();
    ss_pose_curr << topic << "/pose"; topic_pose_curr = ss_pose_curr.str();
    ss_Kp << topic << "/Kp"; topic_Kp = ss_Kp.str();
    ss_Ki << topic << "/Ki"; topic_Ki = ss_Ki.str();
    ss_Kd << topic << "/Kd"; topic_Kd = ss_Kd.str();

    this->__model = _parent;
    this->__joint = this->__model->GetJoint(joint_name);

    this->Load_SimLoop();
    this->Load_NodeROS();

    this->Load_Subscrieber(topic_vel_req, sub_Vel, &ControllerPlugin::OnRosMsg_Vel);
    this->Load_Subscrieber(topic_Kp, sub_Kp, &ControllerPlugin::OnRosMsg_Kp);
    this->Load_Subscrieber(topic_Ki, sub_Ki, &ControllerPlugin::OnRosMsg_Ki);
    this->Load_Subscrieber(topic_Kd, sub_Kd, &ControllerPlugin::OnRosMsg_Kd);

    this->Load_Publisher(topic_vel_curr, pub_Vel);
    this->Load_Publisher(topic_pose_curr, pub_Pose);

    ROS_WARN("%s: Kp: %f | Ki: %f | Kd: %f", joint_name.c_str(), ControllerPID::Kp, ControllerPID::Ki, ControllerPID::Kd);

    // {
    //     if (_sdf->HasElement("topicReducer"))
    //         std::string topicReducer = _sdf->GetElement("topicReducer")->GetValue()->GetAsString();
    //         this->Load_Subscrieber_UInt8(topicReducer, sub_reducer, &ControllerPlugin::OnRosMsg_reducer);
    // }
}



/////////////////////// CONTROL FUNCTIONS ////////////////////////////

void ControllerPlugin::OnUpdate(const common::UpdateInfo &info)
{
    this->OnPerioid();

    if(velocity_req)
        this->__joint->SetForce(0,this->force);
}

void ControllerPlugin::OnPerioid()
{
    number++; if(number > 10) { number = 0;

        this->pose_curr = this->__joint->GetAngle(0).Radian();
        temp_msg.data = pose_curr;
        this->pub_Pose.publish(temp_msg);

        this->velocity_curr = this->__joint->GetVelocity(0);
        temp_msg.data = velocity_curr;
        this->pub_Vel.publish(temp_msg);

        if(velocity_req == 0){
            ControllerPID::ClearBuff();
        }else if(velocity_req == 0xffff){
            this->force = 0;
        }else{
            this->force = ControllerPID::Controll(this->velocity_curr, this->velocity_req);
        }
    }
}

void ControllerPlugin::OnRosMsg_Vel(const std_msgs::Float64ConstPtr &_msg)
{
    this->velocity_req = _msg->data;
}

void ControllerPlugin::OnRosMsg_Kp(const std_msgs::Float64ConstPtr &_msg)
{
    ControllerPID::Kp = _msg->data;
    ControllerPlugin::ReadK();
}

void ControllerPlugin::OnRosMsg_Ki(const std_msgs::Float64ConstPtr &_msg)
{
    ControllerPID::Ki = _msg->data;
    ControllerPlugin::ReadK();
}

void ControllerPlugin::OnRosMsg_Kd(const std_msgs::Float64ConstPtr &_msg)
{
    ControllerPID::Kd = _msg->data;
    ControllerPlugin::ReadK();
}

// void ControllerPlugin::OnRosMsg_reducer(const std_msgs::Float64ConstPtr &_msg)
// {
//     this->force_reductioin_factor = _msg->data;
// }

///////// SYSTEM FUNCTIONS /////////////////////////

void ControllerPlugin::QueueThread()
{
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}

void ControllerPlugin::Load_SimLoop()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ControllerPlugin::OnUpdate, this, _1));
    printf("SimLoop plugin launched succesfully\n");
}

void ControllerPlugin::Load_NodeROS()
{
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    this->rosQueueThread =
    std::thread(std::bind(&ControllerPlugin::QueueThread, this));

    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
    }
}

void ControllerPlugin::Load_Subscrieber(std::string topic,
                                        ros::Subscriber& sub,
                                        void (ControllerPlugin::*function)(const std_msgs::Float64ConstPtr&))
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

// void ControllerPlugin::Load_Subscrieber_UInt8(std::string topic,
//                                         ros::Subscriber& sub,
//                                         void (ControllerPlugin::*function)(const std_msgs::UInt8ConstPtr&))
// {
//     ros::SubscribeOptions so =
//           ros::SubscribeOptions::create<std_msgs::UInt8>(
//           topic,
//           1,
//           boost::bind(function, this, _1),
//           ros::VoidPtr(),
//           &this->rosQueue);
//
//     sub = this->rosNode->subscribe(so);
//
// }

void ControllerPlugin::Load_Publisher(std::string topic, ros::Publisher& pub)
{
    pub = this->rosNode->advertise<std_msgs::Float64>(topic, 100);
}

} //gazebo namespace
