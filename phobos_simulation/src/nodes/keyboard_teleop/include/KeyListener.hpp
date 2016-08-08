#ifndef KEYLISTENER_HPP_
#define KEYLISTENER_HPP_

#include <ros/ros.h>
#include <keyboard/Key.h>

extern void KeyUpCallback();
extern void KeyDownCallback();

class KeyListener{
    int key_number;

    ros::Subscriber sub_key_down;
    ros::Subscriber sub_key_up;

public:
    KeyListener(){};
    KeyListener(ros::NodeHandle* nh);
    ~KeyListener(){};

    void Init(ros::NodeHandle* nh);

    void CallbackDown(const keyboard::Key k);
    void CallbackUp(const keyboard::Key k);

    int ReturnKey() {return key_number;}
};

#endif
