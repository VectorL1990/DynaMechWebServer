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

    mat GetEigenVector(mat Matrix, float EigenValue);

    mat GetFinitRotationAxis(arma::mat TransposeMatrix);

    arma::vec4 CalculateQuatByTransformMatrix(mat TransformMatrix);

    mat GetRMatrixByQuat(vec4 Quat);

    vec GetBMatrix(Vec3d P1, Vec3d P2, mat RMatrix);

    mat GetSkewSymmetricMatrixByVec3(Vec3d Vec);

    mat CalculateTransformMatrixDerivation(double dT, mat NextStepA, mat A);

    mat CalculateAngularVelocity(mat NextStepA, mat A, double dT);

    Vec3d GetAVVector(Vec3d AngularV, Vec3d P1, Vec3d P2);

    mat GetDMatrix(Vec3d FixAVector, mat RMatrix);
};