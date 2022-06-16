#pragma once

#include "Component.h"
#include "Constraint.h"

#include <vector>
#include <map>

using namespace std;

class Model
{
public:
    void UpdateComponentDisplacements(const MatrixXd& DeltaQ);

    void AssignComponentQ(const MatrixXd& InputQ);

    void AssembleComponentQs(MatrixXd& OutputQ);

    vector<Component*> Components;

    vector<Constraint*> Constraints;

    MatrixXd CurrentConstraintArray;

    MatrixXd CurrentJacobianMat;

    MatrixXd CurrentGamaMat;
};