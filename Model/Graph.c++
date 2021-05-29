#include "Graph.h"

void Graph::AssembleConstrainMatrix(int& CurrentNode, vector<int>& VisitedNode)
{
    // Get information of this component here, and assemble constrain matrix

    // 
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

        }
    }
    
    for (<vector<int>::iterator Iter = AdjList[CurrentNode].begin(); Iter != AdjList[CurrentNode].end(); Iter++)
    {
        
    }
    
}