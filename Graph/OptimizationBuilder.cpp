#include "OptimizationBuilder.h"
#include "OptimizationProblem.h"
#include "UAVOptimization.h" 
#include <algorithm>
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

        //  Làm sạch mọi khoảng trắng thừa do lỗi đánh máy file CSV!
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
        to.explosive_required = t.explosize; // Sử dụng đúng tên trường trong struct Target // <-- Gán lượng nổ cần thiết từ dữ liệu gốc
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
            opt.explosive = u.getExplosive(); // <-- Gán lượng nổ của UAV từ dữ liệu gốc
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
            }
            else {
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

    // RÀNG BUỘC: Cho phép nhiều UAV tấn công 1 mục tiêu nếu cần để đủ lượng nổ
    // RÀNG BUỘC: Chọn tập UAV có tổng lượng nổ nhỏ nhất nhưng vẫn >= explosive_required
    for (int j = 0; j < m; ++j) {
        std::vector<int> assignedUAVs;
        for (int i = 0; i < n; ++i) {
            if (best.x[i * m + j] == 1 && prob.uavs[i].explosive > 0) {
                assignedUAVs.push_back(i);
            }
        }

        int k = (int)assignedUAVs.size();
        double minSum = std::numeric_limits<double>::max();
        std::vector<int> bestSubset;

        // Duyệt tất cả tập con (2^k) của assignedUAVs
        for (int mask = 1; mask < (1 << k); ++mask) {
            double sum = 0.0;
            std::vector<int> subset;
            for (int t = 0; t < k; ++t) {
                if (mask & (1 << t)) {
                    sum += prob.uavs[assignedUAVs[t]].explosive;
                    subset.push_back(assignedUAVs[t]);
                }
            }
            if (sum >= prob.targets[j].explosive_required && sum < minSum) {
                minSum = sum;
                bestSubset = subset;
            }
        }

        // Đánh dấu lại các UAV được chọn, loại UAV dư thừa
        for (int idx : assignedUAVs) {
            if (std::find(bestSubset.begin(), bestSubset.end(), idx) != bestSubset.end()) {
                best.x[idx * m + j] = 1;
            }
            else {
                best.x[idx * m + j] = 0;
            }
        }
    }


    // chuẩn bị mảng paths trong best
    best.paths.resize(n, std::vector<std::vector<int>>(m));

    for (int i = 0; i < n; i++)
    {
        const UAVTypeOpt& uav = prob.uavs[i];
        const UnitUAV& unit = unitList.getUnit(uav.unitIndex);
        int startV = graph.findNearestVertex(unit.getX(), unit.getY());

        bool isRecon = (uav.code.rfind("a_A", 0) == 0 && uav.explosive == 0);

        if (isRecon)
        {
            // 1. Lấy danh sách mục tiêu được phân công cho UAV trinh sát này
            std::vector<std::pair<int, double>> assignedTargets; // (j, value)
            for (int j = 0; j < m; j++)
            {
                if (best.x[i * m + j] == 1)
                    assignedTargets.emplace_back(j, prob.targets[j].value);
            }
            std::sort(assignedTargets.begin(), assignedTargets.end(),
                [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                    return a.second > b.second;
                });

            int currentV = startV;
            std::vector<int> allPath; // Lưu toàn bộ đường đi liền mạch
            for (size_t idx = 0; idx < assignedTargets.size(); ++idx)
            {
                int j = assignedTargets[idx].first;
                int endV = prob.targets[j].vertexId;
                std::vector<int> path = graph.shortestPath(currentV, endV);
                if (!path.empty()) {
                    // Ghép đường đi liền mạch (tránh lặp lại điểm đầu)
                    if (!allPath.empty()) path.erase(path.begin());
                    allPath.insert(allPath.end(), path.begin(), path.end());
                    currentV = path.back();
                }
                best.paths[i][j] = path;
            }

            //ĐÁNH DẤU TẤT CẢ MỤC TIÊU MÀ UAV ĐI QUA LÀ ĐÃ TRINH SÁT
            for (int j = 0; j < m; ++j) {
                int tgtVertex = prob.targets[j].vertexId;
                if (std::find(allPath.begin(), allPath.end(), tgtVertex) != allPath.end()) {
                    best.x[i * m + j] = 1; // Đánh dấu là UAV này đã trinh sát mục tiêu j
                }
            }
        }
        else
        {
            // UAV tấn công: giữ nguyên logic cũ
            for (int j = 0; j < m; j++)
            {
                if (best.x[i * m + j] != 1)
                    continue;
                int endV = prob.targets[j].vertexId;
                std::vector<int> path = graph.shortestPath(startV, endV);
                best.paths[i][j] = std::move(path);
            }
        }
    }
    // ràng buộc mỗi mt chỉ phân công cho 1 trinh sát
    for (int j = 0; j < m; ++j) {
        int reconIdx = -1;
        for (int i = 0; i < n; ++i) {
            const auto& uav = prob.uavs[i];
            bool isRecon = (uav.code.rfind("a_A", 0) == 0 && uav.explosive == 0);
            if (isRecon && best.x[i * m + j] == 1) {
                if (reconIdx == -1) {
                    reconIdx = i;
                }
                else {
                    best.x[i * m + j] = 0;
                    best.paths[i][j].clear();
                }
            }
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


