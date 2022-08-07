#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Eigen"
#include <vector>

#include "Constraint.h"

using namespace std;
using namespace Eigen;

enum ComponentConstraintType
{
    Drive,
    Fixed,
    Constrainted,
};

class Component
{
public:

    int component_nb;

    // Position in global coordinate
    double position[3] = {0.0, 0.0, 0.0};

    double euler_angles[3] = {0.0, 0.0, 0.0};

    vector<int> independent_coords_;

    int independent_coord_flags_;

    double coordinates_[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    Vector3d UpVector;

    Vector3d ForwardVector;

    Vector3d RightVector;
};