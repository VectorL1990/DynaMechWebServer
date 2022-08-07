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
    ConstraintTypeEnum constraint_type;

    Component* drive_component;

    Component* passive_component;

    double drive_component_constraint_offset[3] = {0.0, 0.0, 0.0};

    double passive_component_constraint_offset[3] = {0.0, 0.0, 0.0};

    Vector3d component_offset;

    // vector list describes relative rotation of first component
    vector<Vector3d> drive_component_reference_vec_list;

    // vector list describes relative rotation of second component
    vector<Vector3d> passive_component_reference_vec_list;
};