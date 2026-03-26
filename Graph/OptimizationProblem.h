#pragma once
#pragma once
#include <vector>
#include <string>
#include "UAVOptimization.h"


struct TargetOpt
{
    int id;
    std::string code;
    std::string name;
    double value;
    double x, y;
    int vertexId;
    std::string type;
	double explosive_required; // Lượng nổ cần thiết để tiêu diệt mục tiêu

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
    double explosive;
};

class OptimizationProblem
{
public:
    std::vector<TargetOpt> targets;
    std::vector<UAVTypeOpt> uavs;

    AssignmentSolution bestSolution;

    double evaluate(const AssignmentSolution& sol) const;
};
