#ifndef CONTROLLERPID_HPP_
#define CONTROLLERPID_HPP_

#define N 10     // Ilość poprzednich pomiarów, wykożystanych do wyliczenia całki

#include "ros/ros.h"
#include <cstring>

class ControllerPID
{
protected:
    double Kp, Ki, Kd;

    double P, I, D;
    double err[N];

    double output;

    double limit;

public:
    ControllerPID(){
        this->ClearBuff();
        this->Reset();
    };
    ControllerPID(double Kp, double Ki, double Kd){
        this->ClearBuff();
        this->Reset();
        this->Set(Kp, Ki, Kd);
    };
    ~ControllerPID(){};

    void Set(double Kp, double Ki, double Kd) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        ROS_INFO("PID: %f %f %f", this->Kp, this->Ki, this->Kd);
    }

    void ClearBuff(){
        for(int i = 0; i < N; i++){
            err[i] = 0;
        }
        this-> I = 0;
    }

    void Reset(){
        this-> P = 0;
        this-> I = 0;
        this-> D = 0;
        this-> limit = 999999999;
    }

    void ReadK(){ROS_ERROR("%f %f %f", Kp, Ki, Kd);}


    double Controll(double curr_value, double req_value)
    {
        memmove(err+1, err, (N-1)*sizeof(*err));
        err[0] = req_value - curr_value;

        this-> P = err[0];
        this-> I += (err[0] - err[N-1]);
        //this-> D = (err[0] - err[1])/2;
        this-> D = (err[0] + 3*err[1] - 3*err[2] - err[3])/6;

        this->output = P*Kp + I*Ki + D*Kd;


        if(output > limit){
            return limit;
        }else if(output < -limit){
            return -limit;
        }else{
            return output;
        }

    }
};

#endif
