#include "Eigen/Fix"
#include "Eigen/Geometry"

#include <iostream>
#include <chrono>

#include "comm/SerialStream.h"
#include "parts/XYZServo.h"
#include "robot/Leg.h"
#include "robot/Robot.h"

#include "core/Time.h"

#include <cmath>
#include "math/Mathf.h"
#include <cassert>


// #include <Eigen/Dense>

using namespace std;
using namespace Crawler;

SerialStream stream;

int main(){

    Time::Start();

    Robot robot;

    // open servo serial stream
    if(robot.OpenSerialStream("/dev/ttyTHS0")){
        cout << "serial stream open" << endl;
    } else {
        cout << "serial stream open failed" << endl;
        return EXIT_FAILURE;
    }

    // reboot servos
    cout << "reboot servos" << endl;
    for(XYZServo* servo : robot.jointServos){
        servo->reboot();
    }
    Time::Sleep(3.5f);
    
    // move servos to initial position
    cout << "move servos to initial position" << endl;
    for(Leg* leg : robot.legs){
        leg->joints[0]->SetTargetAngle(DEG_2_RADf * 0.0f);
        leg->joints[1]->SetTargetAngle(DEG_2_RADf * -30.0f);
        leg->joints[2]->SetTargetAngle(DEG_2_RADf * -30.0f);
        leg->joints[3]->SetTargetAngle(DEG_2_RADf * -30.0f);
    }
    robot.MoveJointsToTargetSync(2.0f);
    Time::Sleep(2.5f);

    float hip_angle = 0.0f * DEG_2_RADf;
    float foot_xy = 0.14f;
    float foot_z = -0.14f;
    float foot_phi = 0.0f;
    Eigen::Vector3f pos_feet_relative[4] = {
        Eigen::Vector3f(foot_xy * cos(+hip_angle), foot_xy * sin(+hip_angle), foot_z),
        Eigen::Vector3f(foot_xy * cos(-hip_angle), foot_xy * sin(-hip_angle), foot_z),
        Eigen::Vector3f(foot_xy * cos(+hip_angle), foot_xy * sin(+hip_angle), foot_z),
        Eigen::Vector3f(foot_xy * cos(-hip_angle), foot_xy * sin(-hip_angle), foot_z),
    };

    float angles_old[4][4];
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            angles_old[i][j] = 0.0f;   
        }
    }

    Time::Update();
    uint64_t updatePeriodMicros = 1000 * 25;
    uint64_t lastUpdateTimeMicros = 0;

    while(1){
        
        Time::Update();

        // cout << Time::currentTimeMicros << " / " << lastUpdateTimeMicros << endl;

        if(Time::currentTimeMicros - lastUpdateTimeMicros >= updatePeriodMicros){

            float surf_freq = 0.5f;

            float surf_dx = 0.0f;
            float surf_dy = 0.0f;
            float surf_dz = 0.0f;

            // float surf_dx = sin(PI2f * surf_freq * Time::currentTime * 1.0f) * 0.05f;
            // float surf_dy = cos(PI2f * surf_freq * Time::currentTime * 1.0f) * 0.05f;
            // float surf_dz = (sin(PI2f * surf_freq * Time::currentTime)*0.5f+0.5f) * (-0.2f);

            // float surf_rotx = 0.0f;
            // float surf_roty = 0.0f;
            float surf_rotz = 0.0f;

            float surf_rotx = sin(PI2f * surf_freq * Time::currentTime * 1.0f) * 15.0f * DEG_2_RADf;
            float surf_roty = cos(PI2f * surf_freq * Time::currentTime * 1.0f) * 15.0f * DEG_2_RADf;
            // float surf_rotz = sin(PI2f * surf_freq * Time::currentTime * 2.0f) * 15.0f * DEG_2_RADf;

            Eigen::Affine3f surf_transform = Eigen::Affine3f::Identity();
            surf_transform.rotate(Eigen::AngleAxisf(surf_roty, Eigen::Vector3f::UnitY()));
            surf_transform.rotate(Eigen::AngleAxisf(surf_rotx, Eigen::Vector3f::UnitX()));
            surf_transform.rotate(Eigen::AngleAxisf(surf_rotz, Eigen::Vector3f::UnitZ()));
            surf_transform.translate(Eigen::Vector3f(surf_dx, surf_dy, surf_dz));
            Eigen::Affine3f surf_transform_inv = surf_transform.inverse();

            bool all_ik_good = true;
            float angles_out[4][4];
            
            for(int i = 0; i < 4; i++){

                Leg* leg = robot.legs[i];

                Eigen::Vector3f pos_foot_transformed = pos_feet_relative[i];
                pos_foot_transformed = leg->hip_transform * pos_foot_transformed;
                pos_foot_transformed = surf_transform_inv * pos_foot_transformed;
                pos_foot_transformed = leg->hip_transform_inv * pos_foot_transformed;

                // cout << legs[i].hip_transform.matrix() << endl;
                // cout << legs[i].hip_transform_inv.matrix() << endl;
                // cout << surf_transform.matrix() << endl;
                // cout << surf_transform_inv.matrix() << endl;

                bool result = leg->IKSearch(pos_foot_transformed, foot_phi, angles_out[i], angles_old[i]);
                
                if(!result){
                    all_ik_good = false;
                    break;
                }
            }

            if(!all_ik_good){
                cout << "all_ik_good: " << all_ik_good << endl;
            }

            if(all_ik_good){

                // for(int i = 0; i < 4; i++){
                //     for(int j = 0; j < 4; j++){
                //         cout << "a" << i << "/" << j << "=" << (angles_out[i][j]*RAD_2_DEGf) << ", ";
                //     }
                // }
                // cout << endl;

                int k = 0;
                for(int i = 0; i < 4; i++){
                    for(int j = 0; j < 4; j++){
                        float angle = angles_out[i][j];
                        assert(!isnan(angle));
                        robot.legs[i]->joints[j]->SetTargetAngle(angle);
                    }
                }
                robot.MoveJointsToTargetSync(updatePeriodMicros*1.0e-6f);
            }

            lastUpdateTimeMicros = Time::currentTimeMicros;

        }

        usleep(500);

    }

    robot.CloseSerialStream();

    // done
    return EXIT_SUCCESS;

}