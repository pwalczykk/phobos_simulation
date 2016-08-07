#ifndef DRNOISEGEN_HPP_
#define DRNOISEGEN_HPP_

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <phobos_gps_sim/DRNoiseGenConfig.h>

#include <cstdlib>
#include <ctime>
#include <cmath>
#include <complex>

class DRNoiseGen{
    dynamic_reconfigure::Server<phobos_gps_sim::DRNoiseGenConfig> server;
    dynamic_reconfigure::Server<phobos_gps_sim::DRNoiseGenConfig>::CallbackType f;
protected:
    double x_noise;
    double y_noise;
    double z_noise;

    double rand1, rand2;

public:
    DRNoiseGen(){
        f = boost::bind(&DRNoiseGen::Callback, this, _1, _2);
        server.setCallback(f);

        srand(time(NULL));

        x_noise = 0;
        y_noise = 0;
        z_noise = 0;
    }

    void Callback(phobos_gps_sim::DRNoiseGenConfig &config, uint32_t level){
        this->x_noise = config.x_noise;
        this->y_noise = config.y_noise;
        this->z_noise = config.z_noise;
        //ROS_INFO("Standard deviations xyz: %f %f %f", x_noise, y_noise, z_noise);
    }

    double GaussRand(double std_deviation){
        rand1 = (double)rand()/RAND_MAX;
        rand2 = (double)rand()/RAND_MAX;
        //ROS_WARN("%f %f %f", rand1, rand2 ,std_deviation*sqrt(-2*log(rand1))*sin(2*3.14*rand2));
        return std_deviation*sqrt(-2*log(rand1))*sin(2*3.14*rand2);
    }

    void GenerateX(double* input){
        *input += GaussRand(this->x_noise);
    }

    void GenerateY(double* input){
        *input += GaussRand(this->y_noise);
    }

    void GenerateZ(double* input){
        *input += GaussRand(this->z_noise);
    }
};


#endif
