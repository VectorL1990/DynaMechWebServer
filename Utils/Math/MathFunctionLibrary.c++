#include "MathFunctionLibrary.h"

MathFunctionLibrary* MathFunctionLibrary::MathFunctionLibrarySingleton = nullptr;

MathFunctionLibrary* MathFunctionLibrary::GetInstance()
{
    return MathFunctionLibrarySingleton;
}

mat MathFunctionLibrary::GetCoordTransformMatrix(Vec3d V1, Vec3d V2)
{
    mat A = 
    {
        {V1.X * V2.X, V1.X * V2.Y, V1.X * V2.Z},
        {V1.Y * V2.X, V1.Y * V2.Y, V1.Y * V2.Z},
        {V1.Z * V2.X, V1.Z * V2.Y, V1.Z * V2.Z}
    };
    return A;
}

mat MathFunctionLibrary::GetEigenVector(mat Matrix, float EigenValue)
{
    mat SubtractMatrix = Matrix - EigenValue*eye<mat>(Matrix.n_rows, Matrix.n_cols);
    mat ZeroCol(SubtractMatrix.n_cols, 1);
    mat EigenVector = arma::solve(SubtractMatrix, ZeroCol);
    return EigenVector;
}

mat MathFunctionLibrary::GetFinitRotationAxis(arma::mat TransposeMatrix)
{
    arma::mat ZeroCol(TransposeMatrix.n_cols, 1);
    mat EigenVector = solve(TransposeMatrix, ZeroCol);
    return EigenVector;
}

arma::vec4 MathFunctionLibrary::CalculateQuatByTransformMatrix(mat TransformMatrix)
{
    double Trace = arma::trace(TransformMatrix);
    double Lambda0 = sqrt(Trace + 1.0) / 2.0;
    double Lambda1 = (TransformMatrix(2,1) - TransformMatrix(1,2))/4.0;
    double Lambda2 = (TransformMatrix(0,2) - TransformMatrix(2,0))/4.0;
    double Lambda3 = (TransformMatrix(1,0) - TransformMatrix(0,1))/4.0;
    arma::vec4 Quat = {Lambda0, Lambda1, Lambda2, Lambda3};
    return Quat;
}

mat MathFunctionLibrary::GetRMatrixByQuat(vec4 Quat)
{
    mat R =
    {
        {-Quat(1), Quat(0), -Quat(3), Quat(2)},
        {-Quat(2), Quat(3), Quat(0), -Quat(1)},
        {-Quat(3), -Quat(2), Quat(1), Quat(0)}
    };
    return R;
}

vec MathFunctionLibrary::GetBMatrix(Vec3d P1, Vec3d P2, mat RMatrix)
{
    Vec3d U = P2 - P1;
    mat USkewSymmetricMatrix = GetSkewSymmetricMatrixByVec3(U);
    mat PartialMatrix = -2.0*(USkewSymmetricMatrix*RMatrix);
    mat B = arma::join_horiz(arma::eye<mat>(3,3), PartialMatrix);
    return B;
}

mat MathFunctionLibrary::GetSkewSymmetricMatrixByVec3(Vec3d Vec)
{
    mat SSM =
    {
        {0.0, -Vec.Z, Vec.Y},
        {Vec.Z, 0.0, -Vec.X},
        {-Vec.Y, Vec.X, 0.0}
    };
    return SSM;
}

mat MathFunctionLibrary::CalculateTransformMatrixDerivation(double dT, mat NextStepA, mat A)
{
    mat DeltaA = NextStepA - A;
    mat Derivation = DeltaA / dT;
    return Derivation;
}

mat MathFunctionLibrary::CalculateAngularVelocity(mat NextStepA, mat A, double dT)
{
    mat ADerivation = CalculateTransformMatrixDerivation(dT, NextStepA, A);
    mat AngularVelocity = ADerivation*A.t();
    return AngularVelocity;
}

Vec3d MathFunctionLibrary::GetAVVector(Vec3d AngularV, Vec3d P1, Vec3d P2)
{
    Vec3d Offset = P2 - P1;
    mat SSMV = GetSkewSymmetricMatrixByVec3(AngularV);
    colvec3 ColVecOffset = {Offset.X, Offset.Y, Offset.Z};
    mat AVMatrix = SSMV*SSMV*ColVecOffset;
}

mat MathFunctionLibrary::GetDMatrix(Vec3d FixAVector, mat RMatrix)
{
    mat FixASSM = GetSkewSymmetricMatrixByVec3(FixAVector);
    mat PartialMatrix = -2.0*FixASSM*RMatrix;
    mat D = join_horiz(arma::zeros<mat>(3,3), PartialMatrix);
    return D;
}