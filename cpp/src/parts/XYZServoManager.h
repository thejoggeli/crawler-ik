#pragma once

#include "parts/XYZServo.h"
#include "comm/SerialStream.h"

class XYZServoManager {
private:
    XYZServoManager();

public:

    static bool initialized;
    static SerialStream* serialStream;
    static const unsigned int NUM_SERVOS;
    static const uint8_t SERVO_IDS[16];
    static XYZServo* servos[16];
    static XYZServo* servo254;
    
    static void Init(const char* serialPort);
    static void Close();

};
