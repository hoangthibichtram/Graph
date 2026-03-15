#include "OptimizationBuilder.h"
#include <cmath>
#include <fstream>
#include <unordered_map>

static std::unordered_map<std::string, double> loadPij(const std::string& path)
{
    std::unordered_map<std::string, double> mp;

    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cout << "[Pij] ERROR: Cannot open file: " << path << "\n";
        return mp;
    }

    std::string line;
    std::getline(ifs, line); // bỏ header

    while (std::getline(ifs, line))
    {
        if (line.empty()) continue;

        std::cout << "[Pij] RAW LINE: " << line << "\n";

        std::stringstream ss(line);
        std::string uavIdStr, tgtIdStr, pStr;

        std::getline(ss, uavIdStr, ',');
        std::getline(ss, tgtIdStr, ',');
        std::getline(ss, pStr, ',');

        if (uavIdStr.empty() || tgtIdStr.empty() || pStr.empty())
            continue;

        int uavId = std::stoi(uavIdStr);
        int tgtId = std::stoi(tgtIdStr);
        double p = std::stod(pStr);

        std::string key = std::to_string(uavId) + "|" + std::to_string(tgtId);
        mp[key] = p;
    }

    std::cout << "[Pij] Loaded " << mp.size() << " probability entries.\n";
    return mp;
}

OptimizationProblem OptimizationBuilder::build(const UnitUAVList& unitList,
    const Graph& graph)
{
    OptimizationProblem prob;

    // 1. Targets
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

    // 2. Load p_ij từ file
    auto pijMap = loadPij("D:\\VS_Prj\\Graph\\x64\\Debug\\Data\\Probability.csv");

    // 3. UAV types
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
            ut.maxCount = 1;              // nếu 1 dòng = 1 UAV
            ut.maxBudget = u.getCostUsd();
            ut.unitIndex = unitIdx;
            ut.unitName = unit.getUnitName();

            ut.aij.assign(m, 0);
            ut.pij.assign(m, 0.0);

            for (int j = 0; j < m; ++j)
            {
                std::string key = std::to_string(ut.id) + "|" + std::to_string(j);
                auto it = pijMap.find(key);

                if (it != pijMap.end()) {
                    ut.aij[j] = 1;
                    ut.pij[j] = it->second;
                }
                else {
                    ut.aij[j] = 0;
                    ut.pij[j] = 0.0;
                }
            }


            prob.uavs.push_back(ut);
        }
        unitIdx++;
    }

    return prob;
}


