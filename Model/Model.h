#pragma once

#include "BaseComponent.h"
#include "Constraint.h"

class Model
{
public:
    //This is root component which can trace the whole component chain
    BaseComponent* RootComponent;
};