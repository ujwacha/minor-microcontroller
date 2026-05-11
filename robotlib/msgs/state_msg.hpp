#ifndef _STATE_MSG_H
#define _STATE_MSG_H

#include <math.h>
#include "processor_type.h"
#include "math.hpp"

#pragma pack(push, 1)

struct Twist
{
    float vx = 0.0f;
    float w = 0.0f;

    Twist(float _vx, float _w)
        : vx(_vx), w(_w) {}

    Twist() = default;
};

struct Position
{
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;

    Position() = default;

    Position(float _x, float _y, float _theta)
        : x(_x), y(_y), theta(_theta) {}
};

struct Odometry
{
    Position pose;
    Twist twist;
};

class Euler;
class Quaternion;

struct Euler
{
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;

    Euler() = default;

    Euler(float _yaw, float _pitch, float _roll)
        : yaw(_yaw), pitch(_pitch), roll(_roll) {}

    void wrap()
    {
        yaw = wrapAngle(yaw);
        pitch = wrapAngle(pitch);
        roll = wrapAngle(roll);
    }

    Quaternion toQuaternion();
};


#pragma pack(pop)

#endif // _STATE_MSG_H
