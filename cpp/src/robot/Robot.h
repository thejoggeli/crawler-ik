#pragma once

#include "Leg.h"
#include "Joint.h"
#include "parts/XYZServo.h"
#include <vector>
#include "comm/SerialStream.h"

namespace Crawler {

class Robot {
private:

    std::vector<uint16_t> servoGoalBuffer;
    std::vector<uint8_t> servoIds;

public:

    SerialStream* servoSerialStream;
    
    XYZServo* masterServo;
    std::vector<XYZServo*> jointServos;

    std::vector<Leg*> legs;
    std::vector<Joint*> jointsList;

    Robot();
    ~Robot();

    bool OpenSerialStream(const char* device);
    void CloseSerialStream();

    // void Update();
    // void FixedUpdate();

    void MoveJointsToTargetSync(float time);

};

}