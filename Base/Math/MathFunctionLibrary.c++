#include "MathFunctionLibrary.h"

MathFunctionLibrary* MathFunctionLibrary::MathFunctionLibrarySingleton = nullptr;

MathFunctionLibrary* MathFunctionLibrary::GetInstance()
{
    return MathFunctionLibrarySingleton;
}

MatrixXd MathFunctionLibrary::GetBMatrix(MatrixXd RMat, Vector3d JointTranslation)
{
    Matrix3d SSMat = MathFunctionLibrary::GetSkewSymmetricMatrixByVec3(JointTranslation);
    MatrixXd URMat = -2*SSMat*RMat;
    Matrix3d IMat = Matrix3d::Identity(3, 3);
    MatrixXd BMat(3, 7);
    BMat << IMat, URMat;
    return BMat;
}

MatrixXd MathFunctionLibrary::GetDMatrix(Vector3d RefVec, MatrixXd RMat)
{
    Matrix3d ASSMat = MathFunctionLibrary::GetSkewSymmetricMatrixByVec3(RefVec);
    MatrixXd ARMat = -2*ASSMat*RMat;
    Matrix3d ZMat = Matrix3d::Zero(3,3);
    MatrixXd DMat(3,7);
    DMat << ZMat, ARMat;
    return DMat;
}

Matrix3d MathFunctionLibrary::GetSkewSymmetricMatrixByVec3(Vector3d Vec)
{
    Eigen::Matrix3d SSM;
    SSM << 0.0, -Vec.z(), Vec.y(), Vec.z(), 0.0, -Vec.x(), -Vec.y(), Vec.x(), 0.0;
    return SSM;
}

MatrixXd MathFunctionLibrary::GetRMatrix(Quaterniond RigidQuad)
{
    MatrixXd RMat(3, 4);
    RMat(0,0) = -RigidQuad.x();
    RMat(0,1) = RigidQuad.w();
    RMat(0,2) = -RigidQuad.z();
    RMat(0,3) = RigidQuad.y();

    RMat(1,0) = -RigidQuad.y();
    RMat(1,1) = RigidQuad.z();
    RMat(1,2) = RigidQuad.w();
    RMat(1,3) = -RigidQuad.x();

    RMat(2,0) = -RigidQuad.z();
    RMat(2,1) = -RigidQuad.y();
    RMat(2,2) = RigidQuad.x();
    RMat(2,3) = RigidQuad.w();

    return RMat;
}

MatrixXd MathFunctionLibrary::GetJacobianMatrix(Constraint* InConstraint)
{
    MatrixXd RMat1 = GetRMatrix(InConstraint->FirstComponent->Rotation);
    MatrixXd BMat1 = GetBMatrix(RMat1, InConstraint->FirstConstraintRelativeTranslation);
    
    MatrixXd RMat2 = GetRMatrix(InConstraint->SecondComponent->Rotation);
    MatrixXd BMat2 = GetBMatrix(RMat2, InConstraint->SecondConstraintRelativeTranslation);

    MatrixXd Cq1, Cq2;
    if (InConstraint->ConstraintType == ConstraintTypeEnum::SphericalJoint)
    {
        Cq1 = BMat1;
        Cq2 = BMat2;
    }
    else if (InConstraint->ConstraintType == ConstraintTypeEnum::CylindricalJoint)
    {
        MatrixXd DMati1 = GetDMatrix(InConstraint->FirstReferenceVectorList[0], RMat1);
        MatrixXd DMatj1 = GetDMatrix(InConstraint->SecondReferenceVectorList[0], RMat2);
        MatrixXd DMatj2 = GetDMatrix(InConstraint->SecondReferenceVectorList[1], RMat2);
        
        MatrixXd Cqi1 = InConstraint->SecondReferenceVectorList[0].transpose() * DMati1;
        MatrixXd Cqi2 = InConstraint->SecondReferenceVectorList[1].transpose() * DMati1;
        MatrixXd Cqi3 = InConstraint->SecondReferenceVectorList[0].transpose() * BMat1;
        MatrixXd Cqi4 = InConstraint->SecondReferenceVectorList[1].transpose() * BMat1;

        MatrixXd Cqj1 = InConstraint->FirstReferenceVectorList[0].transpose() * DMatj1;
        MatrixXd Cqj2 = InConstraint->FirstReferenceVectorList[0].transpose() * DMatj2;
        MatrixXd Cqj3 = InConstraint->ComponentOffset.transpose() * DMatj1 - InConstraint->SecondReferenceVectorList[0].transpose()*BMat2;
        MatrixXd Cqj4 = InConstraint->ComponentOffset.transpose() * DMatj2 - InConstraint->SecondReferenceVectorList[1].transpose()*BMat2;
    }
}

MatrixXd MathFunctionLibrary::AssembleRowConstraintMatrix(int CompIndex1, int CompIndex2, Quaterniond CompQuat1, Quaterniond CompQuat2,
        Vector3d CompJointTrans1, Vector3d CompJointTrans2, int totalCompNb, ConstraintType Constraint)
{
    int ConstraintRowNb = 0;
    if (Constraint == ConstraintType::SphericalJoint)
    {
        ConstraintRowNb = 3;
        MatrixXd RMat1 = MathFunctionLibrary::GetRMatrix(CompQuat1);
        MatrixXd RMat2 = MathFunctionLibrary::GetRMatrix(CompQuat2);

        MatrixXd BMat1 = MathFunctionLibrary::GetBMatrix(RMat1, CompJointTrans1);
        MatrixXd BMat2 = MathFunctionLibrary::GetBMatrix(RMat2, CompJointTrans2);

        
    }
}

MatrixXd MathFunctionLibrary::AssembleConstraintMatrix(Model* InModel)
{
    for (int i = 0; i < InModel->Components.size(); i++)
    {
        for (int j = 0; j < InModel->Components[i]->Constraints.size(); j++)
        {
            
        }
        
    }
    
}
