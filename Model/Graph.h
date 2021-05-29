#pragma once

#include <vector>
#include <armadillo>

#include "Constraint.h"

using namespace std;
using namespace arma;

struct NodeConnectInfo
{
    int AdjNodeIndex = -1;
    EnumConstraintType ConstraintType = EnumConstraintType::FixJoint;
};

class Graph
{
public:
    void AssembleConstrainMatrix(mat& CMatrix, int& CurrentNode, vector<int>& VisitedNode);

private:
    vector<vector<NodeConnectInfo> > AdjList;
};