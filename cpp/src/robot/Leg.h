#pragma once

#include "Eigen/Fix"
#include "Eigen/Geometry"
#include "Joint.h"

namespace Crawler {

class Leg {

public:

    Crawler::Joint joints[4];

    Eigen::Affine3f hip_translation = Eigen::Affine3f::Identity();
    Eigen::Affine3f hip_rotation = Eigen::Affine3f::Identity();
    Eigen::Affine3f hip_transform = Eigen::Affine3f::Identity();

    Eigen::Affine3f hip_translation_inv = Eigen::Affine3f::Identity();
    Eigen::Affine3f hip_rotation_inv = Eigen::Affine3f::Identity();
    Eigen::Affine3f hip_transform_inv = Eigen::Affine3f::Identity();

    Leg();

    void SetHipTransform(const Eigen::Vector3f& translation, float angle);

    bool IKExact(const Eigen::Vector3f& Q, float phi, float angles_out[4]);
    bool IKSearch(const Eigen::Vector3f& Q, float phi_target, float angles_out[4], float angles_old[4]);
    float IKLoss(float phi_target, float phi_actual, float angles_old[4], float angles_new[4]);

    void FKFoot(); 
    void FKChain();

};

}