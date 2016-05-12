#ifndef DRNOISEGEN_HPP_
#define DRNOISEGEN_HPP_

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <rover_imu/DRNoiseGenConfig.h>

#include <cstdlib>
#include <ctime>
#include <cmath>
#include <complex>

//#define DEG2RAD 0.017452777778

class DRNoiseGen{
    dynamic_reconfigure::Server<rover_imu::DRNoiseGenConfig> server;
    dynamic_reconfigure::Server<rover_imu::DRNoiseGenConfig>::CallbackType f;
protected:
    double orient_noise;
    double ang_vel_noise;
    double lin_acc_noise;

    double rand1, rand2;

public:
    DRNoiseGen(){
        f = boost::bind(&DRNoiseGen::Callback, this, _1, _2);
        server.setCallback(f);

        srand(time(NULL));

        orient_noise = 0;
        ang_vel_noise = 0;
        lin_acc_noise = 0;

    }

    void Callback(rover_imu::DRNoiseGenConfig &config, uint32_t level){
        this->orient_noise = config.orient_noise;
        this->ang_vel_noise = config.ang_vel_noise;
        this->lin_acc_noise = config.lin_acc_noise;
        //ROS_INFO("Standard deviations xyz: %f %f %f", orient_noise, ang_vel_noise, lin_acc_noise);
    }

    double GaussRand(double std_deviation){
        rand1 = (double)rand()/RAND_MAX;
        rand2 = (double)rand()/RAND_MAX;
        //ROS_WARN("%f %f %f", rand1, rand2 ,std_deviation*sqrt(-2*log(rand1))*sin(2*3.14*rand2));
        return std_deviation*sqrt(-2*log(rand1))*sin(2*3.14*rand2);
    }

    void GenerateOrient(double* input){
        *input += GaussRand(this->orient_noise);
    }

    void GenerateAngVel(double* input){
        *input += GaussRand(this->ang_vel_noise);
    }

    void GenerateLinAcc(double* input){
        *input += GaussRand(this->lin_acc_noise);
    }
};


#endif
