#include "Eigen/Fix"
#include "Eigen/Geometry"

#include <iostream>
#include <chrono>

#include "comm/SerialStream.h"
#include "parts/XYZServo.h"
#include "parts/XYZServoManager.h"
#include "robot/Leg.h"

#include "core/Time.h"

#include <cmath>
#include "math/Mathf.h"
#include <cassert>


// #include <Eigen/Dense>

using namespace std;
using namespace Crawler;

SerialStream stream;

int main(){

    XYZServoManager::Init("/dev/ttyTHS0");

    cout << "rebooting servos" << endl;
    for(int i = 0; i < XYZServoManager::NUM_SERVOS; i++){
        XYZServoManager::servos[i]->reboot();
    }
    usleep(3000*1000);

    Leg legs[4];

    int i = 0;
    for(auto& leg : legs){
        leg.joints[0].length = 0.07;
        leg.joints[1].length = 0.07;
        leg.joints[2].length = 0.07;
        leg.joints[3].length = 0.096;
        leg.joints[0].SetServo(XYZServoManager::servos[i++], -1.05f);
        leg.joints[1].SetServo(XYZServoManager::servos[i++], -1.05f);
        leg.joints[2].SetServo(XYZServoManager::servos[i++], -1.05f);
        leg.joints[3].SetServo(XYZServoManager::servos[i++], -1.05f);
    }
    
    float hip_diagonal = 0.155*0.5;
    float hip_dx = hip_diagonal * SQRT2_INVf;
    float hip_dy = hip_diagonal * SQRT2_INVf;
    float hip_dz = 0.0f;

    legs[0].SetHipTransform(Eigen::Vector3f(hip_dx, hip_dy, hip_dz), 45.0f * DEG_2_RADf);
    legs[1].SetHipTransform(Eigen::Vector3f(hip_dx, hip_dy, hip_dz), 135.0f * DEG_2_RADf);
    legs[2].SetHipTransform(Eigen::Vector3f(hip_dx, hip_dy, hip_dz), -135.0f * DEG_2_RADf);
    legs[3].SetHipTransform(Eigen::Vector3f(hip_dx, hip_dy, hip_dz), -45.0f * DEG_2_RADf);

    // Eigen::Vector3f Q(0.20, 0.0, 0.0);
    // float phi = 0.0f * DEG_2_RADf;
    // float angles_out[4];
    // float angles_old[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // int n = 100;
    // float t_start = Time::GetTime();
    // for(int i = 0; i < n ; i++){
    //     bool result = legs[0].IKSearch(Q, phi, angles_out, angles_old);

    // }
    // float t_end = Time::GetTime();
    // float t_duration = t_end - t_start;
    // cout << (t_duration/(float)n*1000.0f) << endl;

    float hip_angle = 0.0f * DEG_2_RADf;
    float foot_xy = 0.15f;
    float foot_z = -0.15f;
    float foot_phi = 0.0f;
    Eigen::Vector3f pos_feet_relative[4] = {
        Eigen::Vector3f(foot_xy * cos(+hip_angle), foot_xy * sin(+hip_angle), foot_z),
        Eigen::Vector3f(foot_xy * cos(-hip_angle), foot_xy * sin(-hip_angle), foot_z),
        Eigen::Vector3f(foot_xy * cos(+hip_angle), foot_xy * sin(+hip_angle), foot_z),
        Eigen::Vector3f(foot_xy * cos(-hip_angle), foot_xy * sin(-hip_angle), foot_z),
    };

    Time::Start();

    float angles_old[4][4];
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            angles_old[i][j] = 0.0f;   
        }
    }

    uint64_t updatePeriodMicros = 1000 * 50;
    uint64_t lastUpdateTimeMicros = 0;

    while(1){
        
        Time::Update();

        // cout << Time::currentTimeMicros << " / " << lastUpdateTimeMicros << endl;

        if(Time::currentTimeMicros - lastUpdateTimeMicros >= updatePeriodMicros){

            float surf_freq = 0.05f;
            float surf_dx = 0.0f; // sin(PI2f * surf_freq * Time::currentTime) * 0.05f;
            float surf_dy = 0.0f; // cos(PI2f * surf_freq * Time::currentTime) * 0.05f;
            float surf_dz = 0.0f;

            float surf_rotx = 0.0f;
            float surf_roty = 0.0f;
            // float surf_rotz = 0.0f;

            // float surf_rotx = sin(PI2f * surf_freq * Time::currentTime * 4.0f) * 30.0f * DEG_2_RADf;
            // float surf_roty = sin(PI2f * surf_freq * Time::currentTime * 3.11f) * 30.0f * DEG_2_RADf;
            float surf_rotz = sin(PI2f * surf_freq * Time::currentTime * 2.55) * 30.0f * DEG_2_RADf;

            Eigen::Affine3f surf_transform = Eigen::Affine3f::Identity();
            surf_transform.rotate(Eigen::AngleAxisf(surf_roty, Eigen::Vector3f::UnitY()));
            surf_transform.rotate(Eigen::AngleAxisf(surf_rotx, Eigen::Vector3f::UnitX()));
            surf_transform.rotate(Eigen::AngleAxisf(surf_rotz, Eigen::Vector3f::UnitZ()));
            surf_transform.translate(Eigen::Vector3f(surf_dx, surf_dy, surf_dz));
            Eigen::Affine3f surf_transform_inv = surf_transform.inverse();

            bool all_ik_good = true;
            float angles_out[4][4];
            
            for(int i = 0; i < 4; i++){

                Eigen::Vector3f pos_foot_transformed = pos_feet_relative[i];
                pos_foot_transformed = legs[i].hip_transform * pos_foot_transformed;
                pos_foot_transformed = surf_transform_inv * pos_foot_transformed;
                pos_foot_transformed = legs[i].hip_transform_inv * pos_foot_transformed;

                // cout << legs[i].hip_transform.matrix() << endl;
                // cout << legs[i].hip_transform_inv.matrix() << endl;
                // cout << surf_transform.matrix() << endl;
                // cout << surf_transform_inv.matrix() << endl;

                bool result = legs[i].IKSearch(pos_foot_transformed, foot_phi, angles_out[i], angles_old[i]);
                
                if(!result){
                    all_ik_good = false;
                    break;
                }
            }

            cout << "all_ik_good: " << all_ik_good << endl;
            if(all_ik_good){

                // for(int i = 0; i < 4; i++){
                //     for(int j = 0; j < 4; j++){
                //         cout << "a" << i << "/" << j << "=" << (angles_out[i][j]*RAD_2_DEGf) << ", ";
                //     }
                // }
                // cout << endl;

                uint16_t positions[16];
                int k = 0;
                for(int i = 0; i < 4; i++){
                    for(int j = 0; j < 4; j++){
                        float angle = angles_out[i][j];
                        assert(!isnan(angle));
                        positions[k] = (uint16_t)(legs[i].joints[j].servo_angle_scale * angle / PIf * 511.0f + 511.0f);
                        k++;
                    }
                }
                uint8_t playtime = updatePeriodMicros/10000;
                XYZServoManager::servo254->setPositionsSync(positions, XYZServoManager::SERVO_IDS, playtime, (uint8_t)XYZServoManager::NUM_SERVOS);
                // store old angles
                for(int i = 0; i < 4; i++){
                    for(int j = 0; j < 4; j++){
                        angles_old[i][j] = angles_out[i][j];
                    }
                }
            }

            lastUpdateTimeMicros = Time::currentTimeMicros;

        }

        usleep(500);

    }


    XYZServoManager::Close();

    // done
    return EXIT_SUCCESS;

}