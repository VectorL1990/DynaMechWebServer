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

    void static KineticAnalyze(Model* InModel, double dT);

    MatrixXd static GetGMatrix(const double EulerAngles[3]);

    MatrixXd static GetRMatrix(Quaterniond RigidQuad);

    MatrixXd static GetQuaternionBMatrix(MatrixXd RMat, Vector3d JointTranslation);

    MatrixXd static GetEulerBMatrix(MatrixXd GMat, const double JointTranslation[3]);

    MatrixXd static GetDMatrix(Vector3d RefVec, MatrixXd RMat);

    MatrixXd static GetEulerDMatrix(const Vector3d& ref_vec, const MatrixXd& g_mat);

    Eigen::Matrix3d static GetSkewSymmetricMatrixByVec3(Vector3d Vec);

    Matrix3d static GetDerivationOfReferenceVector(Constraint *constraint);

    void static AssembleJacobianMatrix(Model* InModel);

    void static AssembleConstraintMatrix(Model* InModel);

    void static AssembleGamaMatrix(Model* InModel);
};