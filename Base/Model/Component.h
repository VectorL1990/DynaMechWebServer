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
    Constraint,
};

class Component
{
public:

    void UpdateComponentRelativeVectors();

    void GetIndependentEulerJacobian(int TotalComponentNb);

    int Id;

    ComponentConstraintType ConstraintType = ComponentConstraintType::Constraint;

    // Position in global coordinate
    double Position[3] = {0.0, 0.0, 0.0};

    double EulerAngles[3] = {0.0, 0.0, 0.0};

    vector<int> IndependentCoords;

    double Coordinates[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

    // Position list of all joints attached, there maybe several joints belonged a single component
    vector<Vector3d> JointTransList;
    
    vector<Constraint*> Constraints;

    Vector3d UpVector;

    Vector3d ForwardVector;

    Vector3d RightVector;
};