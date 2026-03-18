#include "OptimizationProblem.h"
#include "UAVOptimization.h"   


double OptimizationProblem::evaluate(const AssignmentSolution& sol) const
{
    int n = uavs.size();
    int m = targets.size();

    std::vector<double> Sj(m, 1.0);

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < m; ++j)
        {
            int k = i * m + j;
            int xij = sol.x[k];

            if (xij == 1)
            {
                double pij = uavs[i].pij[j];
                Sj[j] *= (1.0 - pij);
            }
        }
    }

    double sumV = 0.0;
    double remain = 0.0;

    for (int j = 0; j < m; ++j)
    {
        double vj = targets[j].value;
        sumV += vj;
        remain += vj * Sj[j];
    }

    double F = sumV - remain;

    double penalty = 0.0;

    for (int i = 0; i < n; ++i)
    {
        double totalCost = 0.0;
        int usedCount = 0;

        for (int j = 0; j < m; ++j)
        {
            int k = i * m + j;
            int xij = sol.x[k];

            if (xij == 1)
            {
                totalCost += uavs[i].costPerAttack;
                usedCount += 1;
            }
        }

        if (totalCost > uavs[i].maxBudget)
            penalty += (totalCost - uavs[i].maxBudget) * 1000.0;

        if (usedCount > uavs[i].maxCount)
            penalty += (usedCount - uavs[i].maxCount) * 1000.0;
    }

    return F - penalty;
}
