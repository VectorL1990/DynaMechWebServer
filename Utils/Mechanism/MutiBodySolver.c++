#include "MutiBodySolver.h"

void MultiBodySolver::AssembleMutiBodyConstraintMatrix(Graph* InGraph, mat& C, int& CurNode, vector<int>& visitedNode)
{
    for (int i = 0; i < AdjList[CurrentNode].size(); i++)
    {
        int Iter = AdjList[CurrentNode][i];
        vector<int>::iterator FindIter = find(VisitedNode.begin(), VisitedNode.end(), Iter);
        if (FindIter == VisitedNode.end())
        {
            // Which means this Adj node is not searched yet
            if (AdjList[Iter].size() == 0)
            {
                // Which means this node has no child node, we 
            }
        }
        else
        {
            // Which means there are constrains between these nodes, assemble C matrix here
            
        }
    }
}
