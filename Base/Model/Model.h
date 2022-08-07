#pragma once

#include "Component.h"
#include "Constraint.h"

#include <vector>
#include <map>

using namespace std;

class Model
{
public:
    void AssignComponentQ(const MatrixXd& InputQ);

    void AssembleComponentQs(MatrixXd& OutputQ);

    void AssembleSingleComponentIndepedentCoordsJacobianMatrix(int component_nb, int& current_assemble_row, MatrixXd& out_matrix);

    void AssembleConstraintJacobianMatrix(MatrixXd& out_matrix);

    void AssembleConstraintMatrix(MatrixXd& out_matrix);

    void AssembleDrivenConstraint(MatrixXd& out_matrix);

    vector<Component*> components_;

    vector<Constraint*> constraints_;

    MatrixXd CurrentConstraintArray;

    MatrixXd CurrentJacobianMat;

    MatrixXd CurrentGamaMat;
};