#include "Joint.h"

namespace Crawler {

Joint::Joint(){

}

void Joint::SetServo(XYZServo* servo, float angle_scale){
    this->servo = servo;
    this->servo_angle_scale = angle_scale;
}

}