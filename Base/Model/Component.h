#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Eigen"
#include <vector>

#include "Constraint.h"

using namespace std;
using namespace Eigen;

class Component
{
public:

    // Position in global coordinate
    Vector3d Position;

    // Rotation in global coordinate
    Quaterniond Rotation;

    // Position list of all joints attached, there maybe several joints belonged a single component
    vector<Vector3d> JointTransList;
    
    vector<Constraint*> Constraints;
};