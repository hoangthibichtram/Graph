#pragma once
#pragma once
#include <vector>
#include <string>
#include "UAVOptimization.h"


struct TargetOpt
{
    int id;
    std::string code;
    double value;
    double x, y;
    int vertexId;
};

struct UAVTypeOpt
{
    int id;
    std::string code;
    double costPerAttack;
    int maxCount;
    double maxBudget;
    int unitIndex;
    std::string unitName;

    std::vector<int> aij;     // khả dụng
    std::vector<double> pij;  // xác suất
};

class OptimizationProblem
{
public:
    std::vector<TargetOpt> targets;
    std::vector<UAVTypeOpt> uavs;

    AssignmentSolution bestSolution;

    double evaluate(const AssignmentSolution& sol) const;
};
