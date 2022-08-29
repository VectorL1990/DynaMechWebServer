#include "Model.h"
#include "../Math/MathConst.h"
#include "../Math/MathFunctionLibrary.h"


void Model::AssignComponentQ(const VectorXd& input_qs)
{
    for (int i = 0; i < input_qs.size(); i++)
    {
        int component_nb = i / NormalCoordDimension;
        int coordinate_nb = i % NormalCoordDimension;
        components_[component_nb]->coordinates_[coordinate_nb] = input_qs[i];
    }    
}

void Model::AssembleComponentQs(VectorXd& output_qs)
{
    output_qs.resize(NormalCoordDimension*components_.size());
    for (int i = 0; i < output_qs.rows(); i++)
    {
        int component_nb = i / NormalCoordDimension;
        int coordinate_nb = i % NormalCoordDimension;
        output_qs(i) = components_[component_nb]->coordinates_[coordinate_nb];
    }
    
}

void Model::AssembleSingleComponentIndepedentCoordsJacobianMatrix(int component_nb, int& current_assemble_row, MatrixXd& out_matrix)
{
    Component* component = components_[component_nb];
    // For example conversion matrix from local to global coordinates is as below:
    // [Ct*Cq - St*Cp*Sq, -Ct*Sq - St*Cp*Cq,  St*Sp ]   [T]
    // ----------------------------------------------------
    // [St*Cq + Ct*Cp*Sq, -St*Sq + Ct*Cp*Cq, -Ct*Sp ] * [P]
    // [Sp*Sq,             Sp*Cq,             Cp    ]   [Q]
    // If the first local coordinate is constrainted, then only the first row is required,

    // (Ct*Cq - St*Cp*Sq)*T + (-Ct*Sq - St*Cp*Cq)*P + (St*Sp)*Q = t

    // constraint matrix is t - deltaT*w - t0 = 0, in global system it becomes:

    // A*(t - deltaT*w - t0) = 0, derivation of 1st coordinate of this equation should be:

    // (-St*Cq - Ct*Cp*Sq)*T + (St*Cq - Ct*Cp*Cq)*P + Ct*Sp*Q

    // derivation of 2rd coordinate shouble be:

    // (St*Sp*Sq)*T + St*Sp*Cq*P + St*Cp*Q

    // derivation of 3rd coordinate shouble be:

    // (-Ct*Sq - St*Cp*Cq)*T + (-Ct*Cq + St*Cp*Sq)*P

    // For example If indeponent coordinates includes angle_1 and andgle_2
    // The Jacobian matrix should be like
    // [0, 0, 0, ... cn1, cn2, cn3, 0,   0,   0,   ... 0, 0, 0]
    // [0, 0, 0, ... cm1, cm2, cm3, 0,   0,   0,   ... 0, 0, 0]
    if (component->independent_coord_flags_ & 0b000001 != 0)
    {
        // Which means the first coordinate is independent
    }
    if (component->independent_coord_flags_ & 0b001000 != 0)
    {
        MatrixXd jacobian_row(1, NormalCoordDimension*components_.size());

        double jacobian_euler_angle_1 = (-sin(component->coordinates_[3])*cos(component->coordinates_[5]) - cos(component->coordinates_[3])*cos(component->coordinates_[4])*sin(component->coordinates_[5]))*component->coordinates_[3] + 
                            (sin(component->coordinates_[3])*sin(component->coordinates_[5]) - cos(component->coordinates_[3])*cos(component->coordinates_[4])*cos(component->coordinates_[5]))*component->coordinates_[4] +
                            (cos(component->coordinates_[3])*sin(component->coordinates_[4]))*component->coordinates_[5];

        double jacobian_euler_angle_2 = (sin(component->euler_angles[0])*sin(component->euler_angles[1])*sin(component->euler_angles[2]))*component->euler_angles[0] +
                            (sin(component->euler_angles[0])*sin(component->euler_angles[1])*cos(component->euler_angles[2]))*component->euler_angles[1] +
                            (sin(component->euler_angles[0])*cos(component->euler_angles[1]))*component->euler_angles[2];

        double jacobian_euler_angle_3 = (-cos(component->euler_angles[0])*sin(component->euler_angles[2]) - sin(component->euler_angles[0])*cos(component->euler_angles[1])*cos(component->euler_angles[2]))*component->euler_angles[0] +
                            (-cos(component->euler_angles[0])*cos(component->euler_angles[2]) + sin(component->euler_angles[0])*cos(component->euler_angles[1])*sin(component->euler_angles[2]))*component->euler_angles[1];

        out_matrix(current_assemble_row, component_nb*NormalCoordDimension + 3) = jacobian_euler_angle_1;
        out_matrix(current_assemble_row, component_nb*NormalCoordDimension + 4) = jacobian_euler_angle_2;
        out_matrix(current_assemble_row, component_nb*NormalCoordDimension + 5) = jacobian_euler_angle_3;
        current_assemble_row += 1;
    }
    if (component->independent_coord_flags_ & 0b010000 != 0)
    {
        double jacobian_euler_angle_1 = (cos(component->euler_angles[0])*cos(component->euler_angles[2]) - sin(component->euler_angles[0])*cos(component->euler_angles[1])*sin(component->euler_angles[2]))*component->euler_angles[0] +
                            (-cos(component->euler_angles[0])*sin(component->euler_angles[2]) - sin(component->euler_angles[0])*cos(component->euler_angles[1])*cos(component->euler_angles[2]))*component->euler_angles[1] +
                            (sin(component->euler_angles[0])*sin(component->euler_angles[1]))*component->euler_angles[2];

        double jacobian_euler_angle_2 = (-cos(component->euler_angles[0])*sin(component->euler_angles[1])*sin(component->euler_angles[2]))*component->euler_angles[0] +
                            (-cos(component->euler_angles[0])*sin(component->euler_angles[1])*cos(component->euler_angles[2]))*component->euler_angles[1] +
                            (-cos(component->euler_angles[0])*cos(component->euler_angles[1]))*component->euler_angles[2];

        double jacobian_euler_angle_3 = (-sin(component->euler_angles[0])*sin(component->euler_angles[1]) + cos(component->euler_angles[0])*cos(component->euler_angles[1])*cos(component->euler_angles[2]))*component->euler_angles[0] +
                            (-sin(component->euler_angles[0])*cos(component->euler_angles[2])-cos(component->euler_angles[0])*cos(component->euler_angles[1])*sin(component->euler_angles[2]))*component->euler_angles[1];

        out_matrix(current_assemble_row, component_nb*NormalCoordDimension + 3) = jacobian_euler_angle_1;
        out_matrix(current_assemble_row, component_nb*NormalCoordDimension + 4) = jacobian_euler_angle_2;
        out_matrix(current_assemble_row, component_nb*NormalCoordDimension + 5) = jacobian_euler_angle_3;
        current_assemble_row += 1;
    }
    if (component->independent_coord_flags_ & 0b100000 != 0)
    {
        double jacobian_euler_angle_1 = 0.0;

        double jacobian_euler_angle_2 = cos(component->euler_angles[1])*sin(component->euler_angles[2])*component->euler_angles[0] +
                            cos(component->euler_angles[1])*cos(component->euler_angles[2])*component->euler_angles[1] +
                            -sin(component->euler_angles[1])*component->euler_angles[2];

        double jacobian_euler_angle_3 = sin(component->euler_angles[1])*cos(component->euler_angles[2])*component->euler_angles[0] +
                            (-sin(component->euler_angles[1])*sin(component->euler_angles[2]))*component->euler_angles[1];

        out_matrix(current_assemble_row, component_nb*NormalCoordDimension + 3) = jacobian_euler_angle_1;
        out_matrix(current_assemble_row, component_nb*NormalCoordDimension + 4) = jacobian_euler_angle_2;
        out_matrix(current_assemble_row, component_nb*NormalCoordDimension + 5) = jacobian_euler_angle_3;
        current_assemble_row += 1;

    }
}

void Model::AssembleAllIndependentCoordsJacobianMatrix(int& current_assemble_row, MatrixXd& out_matrix)
{
    for (int i = 0; i < components_.size(); i++)
    {
        AssembleSingleComponentIndepedentCoordsJacobianMatrix(components_[i]->component_nb, current_assemble_row, out_matrix);
    }
}

void Model::AssembleConstraintJacobianMatrix(int& current_assemble_row, MatrixXd& out_matrix)
{
    for (int i = 0; i < constraints_.size(); i++)
    {
        MatrixXd g_mat_1 = MathFunctionLibrary::GetGMatrix(constraints_[i]->drive_component->euler_angles);
        MatrixXd b_mat_1 = MathFunctionLibrary::GetEulerBMatrix(g_mat_1, constraints_[i]->drive_component_constraint_offset);
        
        MatrixXd g_mat_2 = MathFunctionLibrary::GetGMatrix(constraints_[i]->passive_component->euler_angles);
        MatrixXd b_mat_2 = MathFunctionLibrary::GetEulerBMatrix(g_mat_2, constraints_[i]->passive_component_constraint_offset);

        MatrixXd cq_1, cq_2;
        if (constraints_[i]->constraint_type == ConstraintTypeEnum::SphericalJoint)
        {
            cq_1 = b_mat_1;
            cq_2 = b_mat_2;
            
        }
        else if (constraints_[i]->constraint_type == ConstraintTypeEnum::CylindricalJoint)
        {
            MatrixXd d_mat_i_1 = MathFunctionLibrary::GetEulerDMatrix(constraints_[i]->drive_component_reference_vec_list[0], g_mat_1);
            MatrixXd d_mat_j_1 = MathFunctionLibrary::GetEulerDMatrix(constraints_[i]->passive_component_reference_vec_list[0], g_mat_2);
            MatrixXd d_mat_j_2 = MathFunctionLibrary::GetEulerDMatrix(constraints_[i]->passive_component_reference_vec_list[1], g_mat_2);
            
            MatrixXd cq_i_1 = constraints_[i]->passive_component_reference_vec_list[0].transpose() * d_mat_i_1;
            MatrixXd cq_i_2 = constraints_[i]->passive_component_reference_vec_list[1].transpose() * d_mat_i_1;
            MatrixXd cq_i_3 = constraints_[i]->passive_component_reference_vec_list[0].transpose() * b_mat_1;
            MatrixXd cq_i_4 = constraints_[i]->passive_component_reference_vec_list[1].transpose() * b_mat_1;

            MatrixXd cq_j_1 = constraints_[i]->drive_component_reference_vec_list[0].transpose() * d_mat_j_1;
            MatrixXd cq_j_2 = constraints_[i]->drive_component_reference_vec_list[0].transpose() * d_mat_j_2;
            MatrixXd cq_j_3 = constraints_[i]->component_offset.transpose() * d_mat_j_1 - constraints_[i]->passive_component_reference_vec_list[0].transpose()*b_mat_2;
            MatrixXd cq_j_4 = constraints_[i]->component_offset.transpose() * d_mat_j_2 - constraints_[i]->passive_component_reference_vec_list[1].transpose()*b_mat_2;

            int total_row_nb = cq_i_1.rows() + cq_i_2.rows() + cq_i_3.rows() + cq_i_4.rows();

            cq_1.resize(total_row_nb, NormalCoordDimension);
            cq_2.resize(total_row_nb, NormalCoordDimension);

            cq_1 << cq_i_1,
                    cq_i_2,
                    cq_i_3,
                    cq_i_4;

            cq_2 << cq_j_1,
                    cq_j_2,
                    cq_j_3,
                    cq_j_4;
        }
        current_assemble_row += cq_1.rows();
        // It's important to consider the case that constraint equations may not be in order
        // What it means is that for example constraint A restricts coordinates 3 and 4, and constraint B restricts coordinate 5
        // If we assemble equations in order of 
        out_matrix.block(current_assemble_row, constraints_[i]->drive_component->component_nb*NormalCoordDimension,
                                cq_1.rows(), NormalCoordDimension) << cq_1;

        out_matrix.block(current_assemble_row, constraints_[i]->passive_component->component_nb*NormalCoordDimension,
                                cq_1.rows(), NormalCoordDimension) << cq_2;
    }

}

void Model::AssembleConstraintMatrix(MatrixXd& out_matrix)
{
    int current_assemble_row = 0;
    for (int i = 0; i < constraints_.size(); i++)
    {
        // Constraints may contain some coordinates of components, in such cases constraint matrix(constraint column) may look like
        // [0, c1, c2, 0, c3, 0, ......]T
        // Elements in this matrix may be out of order, so we should get all dependent coordinates for passive component first
        if (constraints_[i]->constraint_type == ConstraintTypeEnum::CylindricalJoint)
        {
            
            int passive_constraint_coordinate_nb = constraints_[i]->passive_component->component_nb;

            double ai_dot_aj1 = constraints_[i]->drive_component_reference_vec_list[0].dot(constraints_[i]->passive_component_reference_vec_list[0]);
            double ai_dot_aj2 = constraints_[i]->drive_component_reference_vec_list[0].dot(constraints_[i]->passive_component_reference_vec_list[1]);
            double aj1_dot_hij = constraints_[i]->passive_component_reference_vec_list[0].dot(constraints_[i]->component_offset);
            double aj2_dot_hij = constraints_[i]->passive_component_reference_vec_list[1].dot(constraints_[i]->component_offset);

            
            out_matrix(current_assemble_row, 0) = ai_dot_aj1;
            out_matrix(current_assemble_row + 1, 0) = ai_dot_aj2;
            out_matrix(current_assemble_row + 2, 0) = aj1_dot_hij;
            out_matrix(current_assemble_row + 3, 0) = aj2_dot_hij;

            current_assemble_row += 4;
        }

    }
    
}

void Model::AssembleGamaMatrix(MatrixXd& out_matrix)
{
    for (int i = 0; i < constraints_.size(); i++)
    {
        /* code */
    }
    
}

void Model::KineticAnalysis()
{
    VectorXd q_array(components_.size() * NormalCoordDimension);

    MatrixXd jacobian_mat(components_.size() * NormalCoordDimension, components_.size() * NormalCoordDimension);
    MatrixXd constraint_mat(components_.size() * NormalCoordDimension, 1);
    int current_loop = 0;
    double t = 0.0;
    while (current_loop <= MaxKineticAnalysisLoopTime)
    {
        AssembleComponentQs(q_array);

        int current_assemble_row = 0;
        AssembleConstraintJacobianMatrix(current_assemble_row, jacobian_mat);
        AssembleAllIndependentCoordsJacobianMatrix(current_assemble_row, jacobian_mat);

        AssembleConstraintMatrix(constraint_mat);

        VectorXd delta_q = jacobian_mat.ldlt().solve(-constraint_mat);

        if (delta_q.norm()<=IterDisplacementEpsilon || constraint_mat.norm() <= ConstraintEpsilon)
        {
            q_array += delta_q;
            AssignComponentQ(q_array);
        }
        else
        {
            // Calculate velocity array and acceleration array
            // 
        }
    }
}
