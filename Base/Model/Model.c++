#include "Model.h"
#include "../Math/MathConst.h"

void Model::UpdateComponentDisplacements(const MatrixXd& DeltaQ)
{
    MatrixXd CurrentQ;
    AssembleComponentQs(CurrentQ);
    CurrentQ = CurrentQ + DeltaQ;
    for (int i = 0; i < CurrentQ.rows(); i++)
    {
        int ComponentNb = i / NormalCoordDimension;
        int CoordinateNb = i % NormalCoordDimension;
        Components[ComponentNb]->Coordinates[CoordinateNb] = CurrentQ[i];
    }
    for (int i = 0; i < Components.size(); i++)
    {
        Components[i]->UpdateComponentRelativeVectors();
    }
}

void Model::AssignComponentQ(const MatrixXd& InputQ)
{
    for (int i = 0; i < InputQ.rows(); i++)
    {
        int ComponentNb = i / NormalCoordDimension;
        int CoordinateNb = i % NormalCoordDimension;
        Components[ComponentNb]->Coordinates[CoordinateNb] = InputQ[i];
    }
    for (int i = 0; i < Components.size(); i++)
    {
        Components[i]->UpdateComponentRelativeVectors();
    }
    
}

void Model::AssembleComponentQs(MatrixXd& OutputQ)
{
    OutputQ.resize(NormalCoordDimension*Components.size());
    for (int i = 0; i < OutputQ.rows(); i++)
    {
        int ComponentNb = i / NormalCoordDimension;
        int CoordinateNb = i % NormalCoordDimension;
        OutputQ(i, 0) = Components[ComponentNb]->Coordinates[CoordinateNb];
    }
    
}
