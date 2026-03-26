#include "OptimizationBuilder.h"
#include "OptimizationProblem.h"
#include "UAVOptimization.h" 
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
    std::getline(ifs, line); // Bỏ qua header đầu tiên

    // Hàm tiện ích nhỏ để xóa khoảng trắng 2 đầu
    auto trim = [](std::string& s) {
        s.erase(0, s.find_first_not_of(" \t\r\n"));
        s.erase(s.find_last_not_of(" \t\r\n") + 1);
        };

    while (std::getline(ifs, line))
    {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string uavCodeStr, tgtIdStr, pStr;

        std::getline(ss, uavCodeStr, ',');
        std::getline(ss, tgtIdStr, ',');
        std::getline(ss, pStr, ',');

        // CHỖ SỬA QUAN TRỌNG NHẤT: Làm sạch mọi khoảng trắng thừa do lỗi đánh máy file CSV!
        trim(uavCodeStr);
        trim(tgtIdStr);
        trim(pStr);

        if (uavCodeStr.empty() || tgtIdStr.empty() || pStr.empty())
            continue;

        try {
            int tgtId = std::stoi(tgtIdStr);
            double p = std::stod(pStr);

            std::string key = uavCodeStr + "|" + std::to_string(tgtId);
            mp[key] = p;

        }
        catch (...) {
            // Lỗi Parse số thì bỏ qua
        }
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
        to.name = t.name;           
        to.value = t.value_usd;
        to.x = t.x;
        to.y = t.y;
        to.vertexId = t.id_vertex;   
        to.type = t.typeVertex;      

        prob.targets.push_back(to);
        // DEBUG TARGET
        std::cout << "[DEBUG] Target " << to.id
            << " name=" << to.name << " pos=(" << to.x << "," << to.y << ") type=" << to.type << "\n";
    }
    std::vector<int> targetIdMap;
    for (const auto& t : targets)
        targetIdMap.push_back(t.target_id);

    // 2. Build UAVTypeOpt list from UnitUAVList
    const auto& units = unitList.getUnits();

    for (const auto& unit : units)
    {
        for (const auto& u : unit.getUAVs())
        {
            UAVTypeOpt opt;
            opt.id = u.getId();
            opt.code = u.getCode();

            // GA parameters (Nguyên bản của bạn, không đổi cost trinh sát gì hết)
            opt.costPerAttack = u.getCostUsd();
            opt.maxBudget = u.getCostUsd() * 3;   // ví dụ: mỗi UAV được dùng tối đa 5 lần
            opt.maxCount = 1; 

            opt.unitIndex = unitList.getUnitIndex(unit.getUnitId());
            opt.unitName = unit.getUnitName();

            // Khởi tạo vector aij/pij theo số mục tiêu
            opt.aij.resize(prob.targets.size(), 1);   // tạm cho phép tấn công tất cả
            opt.pij.resize(prob.targets.size(), 0.0); // sẽ load từ file Pij

            prob.uavs.push_back(opt);
        }
    }

    int m = (int)prob.targets.size();
    int n = (int)prob.uavs.size();
    std::cout << "UAV count = " << n << "\n";

    // 2. Load p_ij từ file
    auto pijMap = loadPij("D:\\VS_Prj\\Graph\\x64\\Debug\\Data\\Probability.csv");
    // 3. Gán Pij vào UAVTypeOpt
    for (auto& uav : prob.uavs)
    {
        for (int j = 0; j < prob.targets.size(); j++)
        {
            std::string key = uav.code + "|" + std::to_string(prob.targets[j].id);
            if (pijMap.count(key)) {
                uav.pij[j] = pijMap[key];
            } else {
              // NẾU GẶP MỤC TIÊU LẠ KHÔNG CÓ TRONG FILE PROBABILITY, GÁN TỶ LỆ TRÚNG 50%
                uav.pij[j] = 0.0; 
            }
        }
    }


    int populationSize = 5;
    int maxGenerations = 3;
    double crossoverRate = 0.8;
    double mutationRate = 0.05;

    UAVGAOptimizer ga(prob, populationSize, maxGenerations, crossoverRate, mutationRate);
    AssignmentSolution best = ga.run();

    // chuẩn bị mảng paths trong best
    best.paths.resize(n, std::vector<std::vector<int>>(m));

    for (int i = 0; i < n; i++)
    {
        const UAVTypeOpt& uav = prob.uavs[i];

        // 1) Lấy đơn vị chứa UAV i
        const UnitUAV& unit = unitList.getUnit(uav.unitIndex);

        // 2) Tìm vertex gần nhất với đơn vị
        int startV = graph.findNearestVertex(unit.getX(), unit.getY());

        for (int j = 0; j < m; j++)
        {
            if (best.x[i * m + j] != 1)
                continue;

            // 3) Tìm vertex gần nhất với target j
            int endV = prob.targets[j].vertexId;   // ⭐ DÙNG ĐÚNG VERTEX CỦA TARGET

           /* int endV = graph.findNearestVertex(prob.targets[j].x,
                prob.targets[j].y);*/

            // 4) Tìm đường đi
            std::vector<int> path = graph.shortestPath(startV, endV);

            // 5) Lưu vào best.paths
            best.paths[i][j] = std::move(path);
        }
    }

    best.unitIndex.resize(n);
    for (int i = 0; i < n; i++)
    {
        best.unitIndex[i] = prob.uavs[i].unitIndex;
    }

    prob.bestSolution.x = best.x;
    prob.bestSolution.fitness = best.fitness;
    prob.bestSolution.paths = best.paths;
    prob.bestSolution.unitIndex = best.unitIndex;
    prob.bestSolution.nUavTypes = best.nUavTypes;
    prob.bestSolution.nTargets = best.nTargets;

    return prob;

}


