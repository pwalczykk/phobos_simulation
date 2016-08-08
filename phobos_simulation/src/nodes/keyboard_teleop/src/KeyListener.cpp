#include "../include/KeyListener.hpp"

KeyListener::KeyListener(ros::NodeHandle* nh)
{
    Init(nh);
}

void KeyListener::Init(ros::NodeHandle* nh)
{
    sub_key_down = nh->subscribe("/keyboard/keydown", 100, &KeyListener::CallbackDown, this);
    sub_key_up = nh->subscribe("/keyboard/keyup", 100, &KeyListener::CallbackUp, this);
}

void KeyListener::CallbackDown(const keyboard::Key k)
{
    this->key_number = k.code;

    KeyDownCallback();
}

void KeyListener::CallbackUp(const keyboard::Key k)
{
    this->key_number = k.code;

    KeyUpCallback();
}
