#pragma once

#include "BasicMathClassExtention.h"

#include <armadillo>

using namespace arma;

class MathFunctionLibrary
{
protected:
    static MathFunctionLibrary* MathFunctionLibrarySingleton;

public:
    static MathFunctionLibrary* GetInstance();

//////// Coordinate related calculation
    mat GetCoordTransformMatrix(Vec3d V1, Vec3d V2);

    Vec3d GetEigenVector(mat Matrix);
};