#pragma once

#include "Component.h"
#include "Constraint.h"

#include <vector>
#include <map>

using namespace std;

class Model
{
public:
    void AssignComponentQ(const VectorXd& InputQ);

    void AssembleComponentQs(VectorXd& OutputQ);

    /*
     * Important!
     * It's not correct that a independent constraint equation is deemed to a specific constraint for a specific coordinate,
     * because a local coordinate is actually affected by several coordinates in global system.
     */
    void AssembleSingleComponentIndepedentCoordsJacobianMatrix(int component_nb, int& current_assemble_row, MatrixXd& out_matrix);

    void AssembleAllIndependentCoordsJacobianMatrix(int& current_assemble_row, MatrixXd& out_matrix);

    /*
     * Keep in mind that a constraint equation does not mean it constraints specific coordinates,
     * for example a cylindrical joint constraints horizontal translational coordinates in local coordinate system,
     * however, it actually constraints all 3 translational coordinates in global coordinate system,
     * conclusion is it's not necessary to put those equations according to coordinate's order
     */
    void AssembleConstraintJacobianMatrix(int& current_assemble_row, MatrixXd& out_matrix);

    void AssembleConstraintMatrix(MatrixXd& out_matrix);

    void AssembleDrivenConstraint(MatrixXd& out_matrix);

    void AssembleGamaMatrix(MatrixXd &out_matrix);

    void KineticAnalysis();

    vector<Component*> components_;

    vector<Constraint*> constraints_;

    MatrixXd CurrentConstraintArray;

    MatrixXd CurrentJacobianMat;

    MatrixXd CurrentGamaMat;
};