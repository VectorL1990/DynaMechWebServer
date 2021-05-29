#pragma once

#include "MathFunctionLibrary.h"

#include "Graph.h"

class MultiBodySolver
{
public:
    void AssembleMutiBodyConstraintMatrix(Graph* InGraph, mat& C, int& CurNode, vector<int>& visitedNode);
};