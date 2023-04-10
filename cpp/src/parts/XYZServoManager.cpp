#include "parts/XYZServoManager.h"


bool XYZServoManager::initialized = false;
SerialStream* XYZServoManager::serialStream = nullptr;
const unsigned int XYZServoManager::NUM_SERVOS = sizeof(XYZServoManager::servos)/sizeof(XYZServo*);
const uint8_t XYZServoManager::SERVO_IDS[16] = {
     7,  8,  9, 10, 
    19, 20,  1,  6,
    15, 16, 17, 18, 
    11, 12, 13, 14, 
};
XYZServo* XYZServoManager::servos[16] = {nullptr};
XYZServo* XYZServoManager::servo254 = nullptr;

XYZServoManager::XYZServoManager(){

}

void XYZServoManager::Init(const char* serialPort){

    if(initialized){
        return;
    }
    initialized = true;

    serialStream = new SerialStream();
    serialStream->open(serialPort, 115200);
    
    for(int i = 0; i < NUM_SERVOS; i++){
        servos[i] = new XYZServo(serialStream, SERVO_IDS[i]);
    }

    servo254 = new XYZServo(serialStream, 254);

}

void XYZServoManager::Close(){
    serialStream->close();
}
