#pragma once
#include "OptimizationTypes.h"
#include "UnitUAVList.h"
#include "Graph.h"


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

class OptimizationBuilder
{
public:
    static OptimizationProblem build(const UnitUAVList& unitList,
        const Graph& graph, const std::string& dataDir);
};