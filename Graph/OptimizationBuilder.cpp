// OptimizationBuilder.cpp — Phiên bản đã xóa hoàn toàn UAV trinh sát
// Lý do: UAV trinh sát (explosive=0) không tham gia bài toán tối ưu GA.
// Chúng đã được xóa khỏi file data_uav_full.csv.
// File này chỉ xử lý UAV tấn công (Chiến đấu + Cảm tử).

#include "OptimizationBuilder.h"
#include "OptimizationProblem.h"
#include "UAVOptimization.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <iostream>
#include <limits>

static std::unordered_map<std::string, double> loadPij(const std::string& path)
{
    std::unordered_map<std::string, double> mp;
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cout << "[Pij] CANH BAO: Khong mo duoc file: " << path << "\n";
        return mp;
    }
    auto trim = [](std::string& s) {
        s.erase(0, s.find_first_not_of(" \t\r\n"));
        s.erase(s.find_last_not_of(" \t\r\n") + 1);
        };
    std::string line;
    std::getline(ifs, line); // Bỏ qua header
    while (std::getline(ifs, line))
    {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string uavCodeStr, tgtIdStr, pStr;
        std::getline(ss, uavCodeStr, ',');
        std::getline(ss, tgtIdStr, ',');
        std::getline(ss, pStr, ',');
        trim(uavCodeStr); trim(tgtIdStr); trim(pStr);
        if (uavCodeStr.empty() || tgtIdStr.empty() || pStr.empty()) continue;
        try {
            int tgtId = std::stoi(tgtIdStr);
            double p = std::stod(pStr);
            mp[uavCodeStr + "|" + std::to_string(tgtId)] = p;
        }
        catch (...) {}
    }
    std::cout << "[Pij] Da tai " << mp.size() << " gia tri xac suat.\n";
    return mp;
}

OptimizationProblem OptimizationBuilder::build(const UnitUAVList& unitList,
    const Graph& graph)
{
    OptimizationProblem prob;

    // PHẦN 1: XÂY DỰNG DANH SÁCH MỤC TIÊU
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
        to.explosive_required = t.explosize;
        to.Priority = t.Priority; // Đọc thẳng từ CSV (cột K)
        prob.targets.push_back(to);         // Chỉ push 1 lần

        std::cout << "[BUILD] Target " << to.id
            << " \"" << to.name << "\""
            << " | vertex=" << to.vertexId
            << " | E_j=" << to.explosive_required
            << " | v_j=" << to.value
            << " | priority=" << to.Priority << "\n";
    }
    int m = (int)prob.targets.size();

    // PHẦN 2: XÂY DỰNG DANH SÁCH UAV TẤN CÔNG
    const auto& units = unitList.getUnits();
    for (const auto& unit : units)
    {
        for (const auto& u : unit.getUAVs())
        {
            // Bỏ qua UAV không có lượng nổ (trinh sát hoặc dữ liệu lỗi)
            if (u.getExplosive() <= 0.0)
            {
                std::cout << "[BUILD] Bo qua UAV " << u.getCode()
                    << " (explosive=0, khong tham gia GA)\n";
                continue;
            }

            UAVTypeOpt opt;
            opt.id = u.getId();
            opt.code = u.getCode();
            opt.explosive = u.getExplosive();
            opt.costPerAttack = u.getCostUsd();
            opt.maxBudget = u.getCostUsd() * 3.0;
            opt.maxCount = 1; // Mỗi UAV chỉ tấn công 1 mục tiêu
            opt.unitIndex = unitList.getUnitIndex(unit.getUnitId());
            opt.unitName = unit.getUnitName();

            opt.aij.resize(m, 1);   // Mặc định khả dụng với mọi mục tiêu
            opt.pij.resize(m, 0.0); // Sẽ load từ Probability.csv
             
            // ── RANGE CHECK: Tính khoảng cách Dijkstra thực tế ──
            // Tìm đỉnh xuất phát của đơn vị (vertex 10000+ đã được thêm bởi ConnectUnitsToGraph)
            int startV = graph.findNearestVertex(unit.getX(), unit.getY());
            double rangeM = u.getRange(); // Đơn vị: meter (sau khi đã sửa CSV)

            std::cout << "[RANGE] UAV " << opt.code
                << " (don vi " << opt.unitName << ")"
                << " | range=" << rangeM / 1000.0 << "km"
                << " | startV=" << startV << "\n";

            for (int j = 0; j < (int)prob.targets.size(); ++j)
            {
                int endV = prob.targets[j].vertexId;

                // Dijkstra: khoảng cách thực tế trên đồ thị (đơn vị meter)
                double dist = graph.shortestPathDistance(startV, endV);

                // So sánh với range → quyết định a_ij
                if (dist > rangeM)
                    opt.aij[j] = 0; 
            }

            prob.uavs.push_back(opt);
            std::cout << "[BUILD] UAV " << opt.code
                << " | don_vi=" << opt.unitName
                << " | explosive=" << opt.explosive
                << " | cost=" << opt.costPerAttack << "\n";
        }
    }

    int n = (int)prob.uavs.size();
    std::cout << "[BUILD] Tong: " << n << " UAV tan cong, " << m << " muc tieu\n";

    // PHẦN 3: GÁN XÁC SUẤT p_{ij} TỪ Probability.csv
    auto pijMap = loadPij("D:\\VS_Prj\\Graph\\x64\\Debug\\Data\\Probability.csv");

    for (auto& uav : prob.uavs)
    {
        for (int j = 0; j < m; ++j)
        {
            std::string key = uav.code + "|" + std::to_string(prob.targets[j].id);
            uav.pij[j] = pijMap.count(key) ? pijMap[key] : 0.0;
        }
    }

    // PHẦN 4: CHẠY GENETIC ALGORITHM
    int    populationSize = 200;
    int    maxGenerations = 500;
    double crossoverRate = 0.85;
    double mutationRate = 0.1;

    UAVGAOptimizer ga(prob, populationSize, maxGenerations, crossoverRate, mutationRate);
    AssignmentSolution best = ga.run();
    std::cout << "[GA] Hoan thanh. Best fitness = " << best.fitness << "\n";

   
    // PHẦN 5: HẬU KỲ — BRUTE-FORCE SUBSET LƯỢNG NỔ
    // Tìm tập con UAV NHỎ NHẤT (tổng explosive nhỏ nhất) đủ >= E_j
    // để giải phóng UAV dư cho mục tiêu khác.
    // O(2^k), k = số UAV/mục tiêu <= 7 → tối đa 128 tập con.
      for (int j = 0; j < m; ++j)
    {
        double Ej = prob.targets[j].explosive_required;
        if (Ej <= 0.0) continue;

        std::vector<int> assignedUAVs;
        for (int i = 0; i < n; ++i)
            if (best.x[i * m + j] == 1 && prob.uavs[i].explosive > 0.0)
                assignedUAVs.push_back(i);

        int k = (int)assignedUAVs.size();
        if (k == 0) continue;

        double           minSum = std::numeric_limits<double>::max();
        std::vector<int> bestSubset;

        for (int mask = 1; mask < (1 << k); ++mask)
        {
            double           sum = 0.0;
            std::vector<int> subset;
            for (int t = 0; t < k; ++t)
            {
                if (mask & (1 << t))
                {
                    sum += prob.uavs[assignedUAVs[t]].explosive;
                    subset.push_back(assignedUAVs[t]);
                }
            }
            if (sum >= Ej && sum < minSum)
            {
                minSum = sum;
                bestSubset = subset;
            }
        }

        for (int iUAV : assignedUAVs)
        {
            bool keep = (std::find(bestSubset.begin(), bestSubset.end(), iUAV)
                != bestSubset.end());
            best.x[iUAV * m + j] = keep ? 1 : 0;
        }

        if (bestSubset.empty())
        {
            std::cout << "[SUBSET] CANH BAO: " << prob.targets[j].name
                << " khong du UAV du luong no E_j=" << Ej << " → huy.\n";
            for (int i = 0; i < n; ++i)
                best.x[i * m + j] = 0;
        }
        else
        {
            std::cout << "[SUBSET] " << prob.targets[j].name
                << " | E_j=" << Ej
                << " | No hop le=" << minSum
                << " | " << bestSubset.size() << " UAV\n";
        }
    }

      // PHẦN 6: TÍNH ĐƯỜNG BAY DIJKSTRA → best.paths[i][j]
       //
       // Với mỗi UAV tấn công i được gán mục tiêu j:
       //   path = shortestPath(startV_đơn_vị → vertexId_mục_tiêu)
    best.paths.resize(n, std::vector<std::vector<int>>(m));

    for (int i = 0; i < n; ++i)
    {
        const UAVTypeOpt& uav = prob.uavs[i];
        const UnitUAV& unit = unitList.getUnit(uav.unitIndex);

        // Đỉnh xuất phát: đỉnh gần nhất với tọa độ căn cứ đơn vị
        int startV = graph.findNearestVertex(unit.getX(), unit.getY());

        for (int j = 0; j < m; ++j)
        {
            if (best.x[i * m + j] != 1) continue; // UAV i không đánh mục tiêu j

            int endV = prob.targets[j].vertexId;

            // Dijkstra: căn cứ đơn vị → mục tiêu j
            std::vector<int> path = graph.shortestPath(startV, endV);

            if (path.empty())
            {
                // Fallback: đường thẳng 2 đỉnh nếu Dijkstra thất bại
                // (Nguyên nhân thường gặp: cạnh chưa nối 2 chiều trong Edge.csv)
                std::cout << "[DIJKSTRA] CANH BAO: Khong tim duoc duong "
                    << startV << "→" << endV
                    << " (UAV " << uav.code << " → " << prob.targets[j].name << ")\n"
                    << "  CHECK: ReadEdgesFile() da goi AddEdge(eV,sV,w) 2 chieu chua?\n";
                path = { startV, endV };
            }

            best.paths[i][j] = path;

            // Log chi tiết đường bay để verify
            std::cout << "[DIJKSTRA] " << uav.code
                << " (" << uav.unitName << ")"
                << " TAN CONG " << prob.targets[j].name
                << " | " << startV << "→" << endV
                << " | " << path.size() << " dinh: [";
            for (int vi = 0; vi < (int)path.size(); ++vi)
                std::cout << path[vi] << (vi + 1 < (int)path.size() ? "→" : "");
            std::cout << "]\n";
        }
    }

    // PHẦN 7: GÁN KẾT QUẢ VÀO prob.bestSolution
    best.unitIndex.resize(n);
    for (int i = 0; i < n; ++i)
        best.unitIndex[i] = prob.uavs[i].unitIndex;

    prob.bestSolution.x = best.x;
    prob.bestSolution.fitness = best.fitness;
    prob.bestSolution.paths = best.paths;
    prob.bestSolution.unitIndex = best.unitIndex;
    prob.bestSolution.nUavTypes = best.nUavTypes;
    prob.bestSolution.nTargets = best.nTargets;
    return prob;
}