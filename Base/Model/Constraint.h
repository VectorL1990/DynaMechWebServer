#pragma once

#include "Component.h"

#include <vector>

using namespace Eigen;
using namespace std;

enum ConstraintTypeEnum
{
    SphericalJoint,
    CylindricalJoint
};

class Constraint
{
public:
    ConstraintTypeEnum ConstraintType;

    Component* FirstComponent;

    Component* SecondComponent;

    double FirstConstraintRelativeTranslation[3] = {0.0, 0.0, 0.0};

    double SecondConstraintRelativeTranslation[3] = {0.0, 0.0, 0.0};

    Vector3d ComponentOffset;

    // vector list describes relative rotation of first component
    vector<Vector3d> FirstReferenceVectorList;

    // vector list describes relative rotation of second component
    vector<Vector3d> SecondReferenceVectorList;
};