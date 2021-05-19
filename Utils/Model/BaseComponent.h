#pragma once
#include <vector>

using namespace std;

class BaseComponent
{
public:
    std::vector<BaseComponent*> ConnectComponents;
};