#pragma once
#include "UAVOptimization.h"
#include "UnitUAVList.h"
#include "Graph.h"

class OptimizationBuilder
{
public:
    static OptimizationProblem build(const UnitUAVList& unitList,
        const Graph& graph);
};
#pragma once
