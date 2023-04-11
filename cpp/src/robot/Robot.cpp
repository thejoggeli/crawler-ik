#include "Robot.h"
#include "math/Mathf.h"
#include <iostream>

using namespace std;

namespace Crawler {

Robot::Robot(){ 

    // create serial stream for servo communication
    servoSerialStream = new SerialStream();

    // all servo ids in order
    // front-left  [hip, knee, knee, knee]
    // back-left   [hip, knee, knee, knee]
    // back-right  [hip, knee, knee, knee]
    // front right [hip, knee, knee, knee]
    servoIds = {
        7,  8,  9, 10, 
        19, 20,  1,  6,
        15, 16, 17, 18, 
        11, 12, 13, 14, 
    };

    // create master servo
    cout << "creating master servo: id=254" << endl;
    masterServo = new XYZServo(servoSerialStream, 254);

    // create joint servos
    for(int i = 0; i < servoIds.size(); i++){
        cout << "creating joint servo: id=" << (int)servoIds[i] << endl;
        XYZServo* servo = new XYZServo(servoSerialStream, servoIds[i]);
        jointServos.push_back(servo);
    }

    // create the legs
    int servoPtr = 0;
    for(int i = 0; i < 4; i++){

        Leg* leg = new Leg();
        legs.push_back(leg);

        // set leg joint lengths
        leg->joints[0]->length = 0.07;
        leg->joints[1]->length = 0.07;
        leg->joints[2]->length = 0.07;
        leg->joints[3]->length = 0.096;

        // create joint servo motors
        leg->joints[0]->SetServo(jointServos[servoPtr++], -1.05f);
        leg->joints[1]->SetServo(jointServos[servoPtr++], -1.05f);
        leg->joints[2]->SetServo(jointServos[servoPtr++], -1.05f);
        leg->joints[3]->SetServo(jointServos[servoPtr++], -1.05f);

        // add all joints of current leg to jointsList
        for(Joint* joint : leg->joints){
            jointsList.push_back(joint);
        }
    }   

    // set legs hip transform
    float hip_diagonal = 0.155*0.5;
    float hip_dx = hip_diagonal * SQRT2_INVf;
    float hip_dy = hip_diagonal * SQRT2_INVf;
    float hip_dz = 0.0f;

    legs[0]->SetHipTransform(Eigen::Vector3f(+hip_dx, +hip_dy, hip_dz), 45.0f * DEG_2_RADf);
    legs[1]->SetHipTransform(Eigen::Vector3f(-hip_dx, +hip_dy, hip_dz), 135.0f * DEG_2_RADf);
    legs[2]->SetHipTransform(Eigen::Vector3f(-hip_dx, -hip_dy, hip_dz), -135.0f * DEG_2_RADf);
    legs[3]->SetHipTransform(Eigen::Vector3f(+hip_dx, -hip_dy, hip_dz), -45.0f * DEG_2_RADf);

    // create position buffer
    for(int i = 0; i < servoIds.size(); i++){
        servoGoalBuffer.push_back(0);
    }

}

Robot::~Robot(){ 
    for(Leg* leg : legs){
        delete leg;
    }
    legs.clear();
    delete masterServo;
}

bool Robot::OpenSerialStream(const char* device){
    int ret = servoSerialStream->open(device, 115200);
    return ret;
}

void Robot::CloseSerialStream(){
    servoSerialStream->close();
}

void Robot::MoveJointsToTargetSync(float time){
    for(int i = 0; i < jointsList.size(); i++){
        Joint* joint = jointsList[i];
        servoGoalBuffer[i] = joint->AngleToXYZ(joint->currentTargetAngle);
    }
    uint8_t playtime = (uint8_t)(time*100.0f);
    masterServo->setPositionsSync(servoGoalBuffer.data(), servoIds.data(), playtime, servoIds.size());
}

}