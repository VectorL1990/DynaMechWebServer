#pragma once

enum EnumConstraintType
{
    FixJoint = 0,
    SphericalJoint = 1,
};

class Constraint
{
public:
    EnumConstraintType ConstraintType = EnumConstraintType::FixJoint;
};