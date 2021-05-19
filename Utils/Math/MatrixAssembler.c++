#include "MatrixAssembler.h"

void MatrixAssembler::AssembleMutiBodyConstraintMatrix(BaseComponent* Component)
{
    for (int i = 0; i < Component->ConnectComponents.size(); i++)
    {

    }
}

void MatrixAssembler::AddConstraintMatrix(EnumConstraintType ContraintType)
{
    if (ContraintType == EnumConstraintType::SphericalJoint)
    {
        
    }
}