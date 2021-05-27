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