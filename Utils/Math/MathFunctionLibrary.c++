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

Vec3d MathFunctionLibrary::GetEigenVector(mat Matrix)
{
    colvec3 Z = 
    mat EigenVector = solve(Matrix, colvec3(0.0));
}