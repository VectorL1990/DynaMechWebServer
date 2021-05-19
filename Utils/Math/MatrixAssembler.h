#pragma once

#include <armadillo>

#include "Model.h"

using namespace arma;

class MatrixAssembler
{
public:
    void AssembleMutiBodyConstraintMatrix(BaseComponent* Component);

    void AddConstraintMatrix(EnumConstraintType ContraintType);
};