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

    Vector3d FirstConstraintRelativeTranslation;

    Vector3d SecondConstraintRelativeTranslation;

    Vector3d ComponentOffset;

    // vector list describes relative rotation of first component
    vector<Vector3d> FirstReferenceVectorList;

    // vector list describes relative rotation of second component
    vector<Vector3d> SecondReferenceVectorList;
};