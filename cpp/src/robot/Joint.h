#pragma once

#include "math/Mathf.h"
#include "parts/XYZServo.h"

namespace Crawler {

class Joint {

public:

    float limit_min = -PIf * 0.5f;
    float limit_max = +PIf * 0.5f;
    float length = 1.0f;
    
    float current_angle = 0.0f;
    float target_angle = 0.0f;

    float servo_angle_scale = 1.0f;
    XYZServo* servo = nullptr;

    Joint();

    void SetServo(XYZServo* servo, float angle_scale);

};    

}