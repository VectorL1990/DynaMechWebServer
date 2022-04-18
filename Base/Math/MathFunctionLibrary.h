#pragma once

#include "eigen3/Eigen/Dense"

#include "Model.h"

using namespace Eigen;


class MathFunctionLibrary
{
protected:
    static MathFunctionLibrary* MathFunctionLibrarySingleton;

public:
    static MathFunctionLibrary* GetInstance();

    MatrixXd static GetBMatrix(MatrixXd RMat, Vector3d JointTranslation);

    MatrixXd static GetDMatrix(Vector3d RefVec, MatrixXd RMat);

    Eigen::Matrix3d static GetSkewSymmetricMatrixByVec3(Vector3d Vec);

    MatrixXd static GetRMatrix(Quaterniond RigidQuad);

    MatrixXd static GetJacobianMatrix(Component* Component1, Component* Component2, Constraint* Joint);

    MatrixXd static AssembleRowConstraintMatrix(int CompIndex1, int CompIndex2, Quaterniond CompQuat1, Quaterniond CompQuat2,
        Vector3d CompJointTrans1, Vector3d CompJointTrans2, int totalCompNb, ConstraintType Constraint);

    MatrixXd static AssembleConstraintMatrix(Model* InModel);
};