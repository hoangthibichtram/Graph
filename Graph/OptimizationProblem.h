#pragma once
#pragma once
#include <vector>
#include <string>

struct TargetOpt
{
    int id;
    std::string code;
    double value;
    double x, y;
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

    struct Solution {
        std::vector<int> x;   // xij nhị phân, kích thước = n*m
        double fitness;
        std::vector<std::vector<std::vector<int>>> paths;
        int nUavTypes = 0;
        int nTargets = 0;
        std::vector<int> unitIndex;
    };

    Solution bestSolution;

    double evaluate(const Solution& sol) const;
};
