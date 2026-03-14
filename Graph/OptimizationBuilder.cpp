#include "OptimizationBuilder.h"
#include <cmath>

OptimizationProblem OptimizationBuilder::build(const UnitUAVList& unitList,
    const Graph& graph)
{
    OptimizationProblem prob;

    // 1. Lấy danh sách target
    const auto& targets = graph.GetTargets();
    for (const auto& t : targets)
    {
        TargetOpt to;
        to.id = t.target_id;
        to.code = t.code;
        to.value = t.value_usd;
        to.x = t.x;
        to.y = t.y;
        prob.targets.push_back(to);
    }

    int m = (int)prob.targets.size();

    // 2. Lấy danh sách UAV
    const auto& units = unitList.getUnits();
    int uavTypeIndex = 0;
    int unitIdx = 0;
    for (const auto& unit : units)
    {
        for (const auto& u : unit.getUAVs())
        {
            UAVTypeOpt ut;
            ut.id = uavTypeIndex++;
            ut.code = u.getCode();
            ut.costPerAttack = u.getCostUsd();
            ut.maxCount = 1;
            ut.maxBudget = u.getCostUsd();

            // ⭐ GÁN ĐƠN VỊ CHO UAV
            ut.unitIndex = unitIdx;                 // index đơn vị
            ut.unitName = unit.getUnitName();      // tên đơn vị: SQ1, SQ2, SQ3

            ut.aij.resize(m);
            ut.pij.resize(m);

            for (int j = 0; j < m; ++j)
            {
                ut.aij[j] = 1;
                ut.pij[j] = 0.7;
            }

            prob.uavs.push_back(ut);
        }
        unitIdx++;
    }


    return prob;
}

