#pragma once

#include <vector>
#include <string>
#include "OptimizationProblem.h"   // dùng lại TargetOpt, UAVTypeOpt, OptimizationProblem

// Một nghiệm (cá thể) của GA: ma trận x_ij
struct AssignmentSolution {
    int nUavTypes;
    int nTargets;
    std::vector<int> x;
    double fitness;
    std::vector<int> unitIndex;

    std::vector<std::vector<std::vector<int>>> paths;

    int at(int i, int j) const {
        return x[i * nTargets + j];
    }
    int& at(int i, int j) {
        return x[i * nTargets + j];
    }
};

// Lớp GA
class UAVGAOptimizer
{
public:
    UAVGAOptimizer(const OptimizationProblem& problem,
        int populationSize,
        int maxGenerations,
        double crossoverRate,
        double mutationRate);

    AssignmentSolution run();

private:
    const OptimizationProblem& prob_;
    int popSize_;
    int maxGen_;
    double pc_;
    double pm_;

    std::vector<AssignmentSolution> population_;

    void initPopulation();
    void evaluate(AssignmentSolution& sol);
    AssignmentSolution selectParent();
    AssignmentSolution crossover(const AssignmentSolution& p1, const AssignmentSolution& p2);
    void mutate(AssignmentSolution& child);
    void repair(AssignmentSolution& sol);
};
