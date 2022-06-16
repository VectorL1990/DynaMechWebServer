#include "MathFunctionLibrary.h"
#include "MathConst.h"

MathFunctionLibrary* MathFunctionLibrary::MathFunctionLibrarySingleton = nullptr;

MathFunctionLibrary* MathFunctionLibrary::GetInstance()
{
    return MathFunctionLibrarySingleton;
}

void MathFunctionLibrary::KineticAnalyze(Model* InModel, double dT)
{
    while (1)
    {
        AssembleJacobianMatrix(InModel);
        AssembleConstraintMatrix(InModel);
        MatrixXd DeltaQ = InModel->CurrentJacobianMat.ldlt().solve(-InModel->CurrentConstraintArray);
        if (DeltaQ.norm() <= IterDisplacementEpsilon || InModel->CurrentConstraintArray.norm() <= ConstraintEpsilon)
        {
            // Which means all components have already satisfied constraint equations
            break;
        }
        else
        {
            MatrixXd Velocity = DeltaQ / dT;
            AssembleGamaMatrix(InModel);
            MatrixXd Acceleration = InModel->CurrentJacobianMat.ldlt().solve(InModel->CurrentGamaMat);
            MatrixXd CurrentDisplacement;
            InModel->AssembleComponentQs(CurrentDisplacement);
            MatrixXd NextDisplacement = CurrentDisplacement + Velocity*dT + 0.5*dT*dT*Acceleration;
            InModel->UpdateComponentDisplacements(NextDisplacement);
        }
    }
}

MatrixXd MathFunctionLibrary::GetGMatrix(const double EulerAngles[3])
{
    MatrixXd GMat(3, 3);

    GMat(0, 0) = 0.0f;
    GMat(0, 1) = cos(EulerAngles[0]);
    GMat(0, 2) = sin(EulerAngles[0]) * sin(EulerAngles[1]);

    GMat(1, 0) = 0.0;
    GMat(1, 1) = sin(EulerAngles[0]);
    GMat(1, 2) = -cos(EulerAngles[0]) * sin(EulerAngles[1]);

    GMat(2, 0) = 1.0;
    GMat(2, 1) = 0.0;
    GMat(2, 2) = cos(EulerAngles[1]);

    return GMat;
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

MatrixXd MathFunctionLibrary::GetQuaternionBMatrix(MatrixXd RMat, Vector3d JointTranslation)
{
    Matrix3d SSMat = MathFunctionLibrary::GetSkewSymmetricMatrixByVec3(JointTranslation);
    MatrixXd URMat = -2*SSMat*RMat;
    Matrix3d IMat = Matrix3d::Identity(3, 3);
    MatrixXd BMat(3, 7);
    BMat << IMat, URMat;
    return BMat;
}

MatrixXd MathFunctionLibrary::GetEulerBMatrix(MatrixXd GMat, Vector3d JointTranslation)
{
    MatrixXd BMat(3, 6);
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

void MathFunctionLibrary::AssembleJacobianMatrix(Model* InModel)
{
    int TotalConstraintMatRow = 0;
    vector<MatrixXd> AllConstraintRowMats;
    for (int i = 0; i < InModel->Constraints.size(); i++)
    {
        MatrixXd GMat1 = GetGMatrix(InModel->Constraints[i]->FirstComponent->EulerAngles);
        MatrixXd BMat1 = GetEulerBMatrix(GMat1, InModel->Constraints[i]->FirstConstraintRelativeTranslation);
        
        MatrixXd GMat2 = GetGMatrix(InModel->Constraints[i]->SecondComponent->EulerAngles);
        MatrixXd BMat2 = GetQuaternionBMatrix(GMat2, InModel->Constraints[i]->SecondConstraintRelativeTranslation);

        int ConstraintMatCol, ConstraintMatRow = 0;
        MatrixXd Cq1, Cq2;
        if (InModel->Constraints[i]->ConstraintType == ConstraintTypeEnum::SphericalJoint)
        {
            Cq1 = BMat1;
            Cq2 = BMat2;
            ConstraintMatCol = Cq1.cols();
            ConstraintMatRow = Cq1.rows();
            TotalConstraintMatRow += ConstraintMatRow;
        }
        else if (InModel->Constraints[i]->ConstraintType == ConstraintTypeEnum::CylindricalJoint)
        {
            MatrixXd DMati1 = GetDMatrix(InModel->Constraints[i]->FirstReferenceVectorList[0], RMat1);
            MatrixXd DMatj1 = GetDMatrix(InModel->Constraints[i]->SecondReferenceVectorList[0], RMat2);
            MatrixXd DMatj2 = GetDMatrix(InModel->Constraints[i]->SecondReferenceVectorList[1], RMat2);
            
            MatrixXd Cqi1 = InModel->Constraints[i]->SecondReferenceVectorList[0].transpose() * DMati1;
            MatrixXd Cqi2 = InModel->Constraints[i]->SecondReferenceVectorList[1].transpose() * DMati1;
            MatrixXd Cqi3 = InModel->Constraints[i]->SecondReferenceVectorList[0].transpose() * BMat1;
            MatrixXd Cqi4 = InModel->Constraints[i]->SecondReferenceVectorList[1].transpose() * BMat1;

            MatrixXd Cqj1 = InModel->Constraints[i]->FirstReferenceVectorList[0].transpose() * DMatj1;
            MatrixXd Cqj2 = InModel->Constraints[i]->FirstReferenceVectorList[0].transpose() * DMatj2;
            MatrixXd Cqj3 = InModel->Constraints[i]->ComponentOffset.transpose() * DMatj1 - InModel->Constraints[i]->SecondReferenceVectorList[0].transpose()*BMat2;
            MatrixXd Cqj4 = InModel->Constraints[i]->ComponentOffset.transpose() * DMatj2 - InModel->Constraints[i]->SecondReferenceVectorList[1].transpose()*BMat2;

            ConstraintMatCol = Cqi1.cols();
            ConstraintMatRow = Cqi1.rows() + Cqi2.rows() + Cqi3.rows() + Cqi4.rows();

            Cq1.resize(4*ConstraintMatRow, ConstraintMatCol);
            Cq2.resize(4*ConstraintMatRow, ConstraintMatCol);

            Cq1 << Cqi1,
                   Cqi2,
                   Cqi3,
                   Cqi4;

            Cq2 << Cqj1,
                   Cqj2,
                   Cqj3,
                   Cqj4;
        }
        MatrixXd RowConstraintMat(ConstraintMatRow, InModel->Components.size()*ConstraintMatCol);

        for (int j = 0; j < InModel->Components.size(); j++)
        {
            if (InModel->Constraints[i]->FirstComponent == InModel->Components[j])
            {
                RowConstraintMat << Cq1;
            }
            else if (InModel->Constraints[i]->SecondComponent == InModel->Components[j])
            {
                RowConstraintMat << Cq2;
            }
            else
            {
                // Construct zero matrix
                MatrixXd ZeroMatrix = MatrixXd::Zero(ConstraintMatRow, ConstraintMatCol);
                RowConstraintMat << ZeroMatrix;
            }
        }
        AllConstraintRowMats.push_back(RowConstraintMat);
        TotalConstraintMatRow += ConstraintMatRow;
    }
    


    if (AllConstraintRowMats.size() > 0)
    {
        InModel->CurrentJacobianMat.resize(TotalConstraintMatRow, AllConstraintRowMats[0].cols());
        for (int i = 0; i < AllConstraintRowMats.size(); i++)
        {
            InModel->CurrentJacobianMat << AllConstraintRowMats[i];
        }
        
    }
}


void MathFunctionLibrary::AssembleConstraintMatrix(Model* InModel)
{
    for (int i = 0; i < InModel->Constraints.size(); i++)
    {
        MatrixXd ConstraintMat;
        if (InModel->Constraints[i]->ConstraintType == ConstraintTypeEnum::SphericalJoint)
        {
            ConstraintMat.resize(3, 1);
            ConstraintMat = (InModel->Constraints[i]->FirstComponent->Position + InModel->Constraints[i]->FirstConstraintRelativeTranslation) - 
                (InModel->Constraints[i]->SecondComponent->Position + InModel->Constraints[i]->SecondConstraintRelativeTranslation);
        }
        else if (InModel->Constraints[i]->ConstraintType == ConstraintTypeEnum::CylindricalJoint)
        {
            ConstraintMat.resize(4, 1);
            ConstraintMat << InModel->Constraints[i]->FirstReferenceVectorList[0].transpose() * InModel->Constraints[i]->SecondReferenceVectorList[0];
            ConstraintMat << InModel->Constraints[i]->FirstReferenceVectorList[0].transpose() * InModel->Constraints[i]->SecondReferenceVectorList[1];
            ConstraintMat << InModel->Constraints[i]->SecondReferenceVectorList[0].transpose() * InModel->Constraints[i]->ComponentOffset;
            ConstraintMat << InModel->Constraints[i]->SecondReferenceVectorList[1].transpose() * InModel->Constraints[i]->ComponentOffset;
        }
    }
       
}

void MathFunctionLibrary::AssembleGamaMatrix(Model* InModel)
{

}