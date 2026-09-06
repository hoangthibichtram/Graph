#pragma once
#include <vector>
#include <string>

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
    int priority;

};

struct UAVTypeOpt
{
    int id;
    std::string code;
    int maxCount;
    int unitIndex;
    double ValuePerAttack;
    std::string unitName;
    std::vector<int> aij;     // khả dụng
    std::vector<double> pij;  // xác suất
    double explosive;
};

struct AssignmentSolution {
    int nUavTypes{};
    int nTargets{};
    std::vector<int> x;
    double fitness{};
    std::vector<int> unitIndex;
    std::vector<std::vector<std::vector<int>>> paths;

    int at(int i, int j) const { return x[i * nTargets + j]; }
    int& at(int i, int j) { return x[i * nTargets + j]; }
};

class OptimizationProblem
{
public:
    std::vector<TargetOpt> targets;
    std::vector<UAVTypeOpt> uavs;

    AssignmentSolution bestSolution;
};
