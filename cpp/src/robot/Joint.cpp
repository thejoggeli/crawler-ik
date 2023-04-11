#include "Joint.h"
#include "parts/XYZServo.h"
#include "math/Mathf.h"

namespace Crawler {

Joint::Joint(){

}

void Joint::SetServo(XYZServo* servo, float angleScale){
    this->servo = servo;
    this->servoAngleScale = angleScale;
}

void Joint::ReadCurrentAngle(){
    // TODO
    // uint16_t xyzAngle = servo->
}

void Joint::SetTargetAngle(float angle){
    lastTargetAngle = currentTargetAngle;
    currentTargetAngle = angle;
}

uint16_t Joint::AngleToXYZ(float angle){
    return (uint16_t)(servoAngleScale * angle / PIf * 511.0f + 511.0f);
}

}