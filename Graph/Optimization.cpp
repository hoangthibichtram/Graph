#include "Optimization.h"
#include <random>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric> 
#include <chrono>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <limits>


//hàm phụ trợ
static std::mt19937& rng()
{
    static std::mt19937 gen(
        std::random_device{}() ^
        (uint32_t)std::chrono::steady_clock::now().time_since_epoch().count()
    );
    return gen;
}

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
        std::string probIdStr, uavCodeStr, tgtIdStr, pStr;
        std::getline(ss, probIdStr, ',');
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

// Khởi tạo tham số GA (kích thước quần thể, số thế hệ, tỉ lệ lai/đột biến)
UAVGAOptimizer::UAVGAOptimizer(const OptimizationProblem& problem,
    int populationSize,
    int maxGenerations,
    double crossoverRate,
    double mutationRate)
    : prob_(problem)
    , popSize_(populationSize)
    , maxGen_(maxGenerations)
    , pc_(crossoverRate)
    , pm_(mutationRate)
{
}

//GA

// Khởi tạo quần thể: x[i,j] = số UAV loại i tấn công mục tiêu j
void UAVGAOptimizer::initPopulation()
{
    int n = (int)prob_.uavs.size();    // số loại UAV (thực chất là số lượng từng con UAV)
    int m = (int)prob_.targets.size(); // số mục tiêu

    if (n <= 0 || m <= 0)
    {
        population_.clear();
        population_.resize(1);
        population_[0] = { n, m, std::vector<int>(n * m, 0), 0.0 };
        return;
    }

    population_.clear();
    population_.resize(popSize_);

    // Sắp xếp mục tiêu theo Priority tăng dần (Priority=1 quan trọng nhất)
    std::vector<int> targetByPriority(m);
    std::iota(targetByPriority.begin(), targetByPriority.end(), 0);
    std::sort(targetByPriority.begin(), targetByPriority.end(), [&](int a, int b) {
        return prob_.targets[a].priority < prob_.targets[b].priority;
        });

    std::uniform_int_distribution<int> targetDist(0, m - 1);
    std::uniform_real_distribution<double> probDist(0.0, 1.0);

    for (int k = 0; k < popSize_; ++k)
    {
        AssignmentSolution sol;
        sol.nUavTypes = n;
        sol.nTargets = m;
        sol.x.assign(n * m, 0);
        if (probDist(rng()) < 0.25)
        {
            // Xáo trộn thứ tự UAV để các cá thể "thông minh" đa dạng hơn
            std::vector<int> uavOrder(n);
            std::iota(uavOrder.begin(), uavOrder.end(), 0);
            for (int s = n - 1; s > 0; --s) {
                std::uniform_int_distribution<int> pick(0, s);
                std::swap(uavOrder[s], uavOrder[pick(rng())]);
            }

            // ── Khởi tạo "định hướng": gán UAV theo mục tiêu ưu tiên cao ──
            // Duyệt từng mục tiêu theo thứ tự ưu tiên,
            // cố gắng tìm UAV phù hợp (a_{ij}=1) chưa được gán
            for (int jIdx = 0; jIdx < m; ++jIdx)
            {
                int j = targetByPriority[jIdx];
                double accumulated = 0.0;
                for (int orderIdx = 0; orderIdx < n; ++orderIdx)
                {
                    int i = uavOrder[orderIdx];
                    // Chỉ gán nếu: khả dụng + chưa gán mục tiêu nào (maxCount=1)
                    if (prob_.uavs[i].aij[j] == 0) continue;
                    int alreadyUsed = 0;
                    for (int jj = 0; jj < m; ++jj)
                        alreadyUsed += sol.at(i, jj);
                    if (alreadyUsed >= prob_.uavs[i].maxCount) continue;

                    sol.at(i, j) = 1;
                    accumulated += prob_.uavs[i].explosive;

                    // Gán đủ lượng nổ thì dừng (không gán thêm UAV thừa)
                    if (accumulated >= prob_.targets[j].explosive_required) break;
                }
            }
        }
        else
        {
            // ── Khởi tạo ngẫu nhiên: mỗi UAV chọn 1 mục tiêu ngẫu nhiên ──
            for (int i = 0; i < n; ++i)
            {
                // 90% khả năng gán nhiệm vụ (10% để UAV "nghỉ" → đa dạng gen)
                if (probDist(rng()) < 0.9)
                {
                    int j = targetDist(rng());
                    if (prob_.uavs[i].aij[j] != 0)
                        sol.at(i, j) = 1;
                }
            }
        }
        // Bắt buộc repair → evaluate cho mọi cá thể ban đầu
        repair(sol);
        evaluate(sol);
        population_[k] = sol;
    }
}
      

// Chọn lọc roulette
AssignmentSolution UAVGAOptimizer::selectParent()
{
    // Tìm giá trị fitness nhỏ nhất trong quần thể
    double minFit = population_[0].fitness;

    for (auto& s : population_)
        if (s.fitness < minFit) minFit = s.fitness;

    // Shift về dương: shifted = fitness - minFit + epsilon
    const double eps = 1e-9;
    double sumShifted = 0.0;
    std::vector<double> shifted(population_.size());
    for (int k = 0; k < (int)population_.size(); ++k)
    {
        shifted[k] = population_[k].fitness - minFit + eps;
        sumShifted += shifted[k];
    }
    // Quay roulette trên fitness đã shift
    std::uniform_real_distribution<double> dist(0.0, sumShifted);
    double r = dist(rng());

    double acc = 0.0;
    for (int k = 0; k < (int)population_.size(); ++k)
    {
        acc += shifted[k];
        if (acc >= r) return population_[k];
    }
    return population_.back();
}

//lai ghép
AssignmentSolution UAVGAOptimizer::crossover(const AssignmentSolution& p1, const AssignmentSolution& p2)
{
    AssignmentSolution child;
    child.nUavTypes = p1.nUavTypes;
    child.nTargets = p1.nTargets;
    child.x = p1.x;
    child.fitness = 0.0;

    int L = (int)child.x.size();
    if (L <= 0) return child;

    std::uniform_int_distribution<int> posDist(0, L - 1);
    std::uniform_real_distribution<double> prob(0.0, 1.0);

    if (prob(rng()) < pc_)
    {
        int cut = posDist(rng());
        for (int k = cut; k < L; ++k)
        {
            child.x[k] = p2.x[k];
        }
    }
    return child;
}

//Đột biến
void UAVGAOptimizer::mutate(AssignmentSolution& child)
{
    int n = child.nUavTypes;
    int m = child.nTargets;
    std::uniform_real_distribution<double> prob(0.0, 1.0);

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < m; ++j)
        {
            // Ràng buộc cứng: UAV không khả dụng → không thể gán
            if (prob_.uavs[i].aij[j] == 0)
            {
                child.at(i, j) = 0;
                continue;
            }
            if (prob(rng()) < pm_)
            {
                int& xij = child.at(i, j);
                xij = 1 - xij; // Đảo bit
            }
        }
    }
}


void UAVGAOptimizer::repair(AssignmentSolution& sol)
{
    if (sol.x.size() != sol.nUavTypes * sol.nTargets)
        sol.x.assign(sol.nUavTypes * sol.nTargets, 0);

    int n = sol.nUavTypes;
    int m = sol.nTargets;
    if (n <= 0 || m <= 0) return;

    // 1. Ràng buộc số lượng & ngân sách cho từng UAV 
    for (int i = 0; i < n; ++i)
    {

        const auto& u = prob_.uavs[i];
        int maxCount = u.maxCount;      // = 1 trong bài toán này
        double value = u.ValuePerAttack;

        // Thu thập các mục tiêu UAV i đang được gán, tính combined score
        struct ScoredTarget {
            int    j;
            double score; // = efficiency * priorityWeight
        };
        std::vector<ScoredTarget> assigned;

        for (int j = 0; j < m; ++j)
        {
            if (sol.at(i, j) != 1) continue;

            double vj = prob_.targets[j].value;
            double pij = u.pij[j];
            int    prio = (prob_.targets[j].priority > 0) ? prob_.targets[j].priority : 1;
            double combined = vj * pij / (double)prio;      // score = giá trị × xác suất / độ ưu tiên

            assigned.push_back({ j, combined });
        }

        if ((int)assigned.size() > maxCount)
        {
            std::sort(assigned.begin(), assigned.end(),
                [](const ScoredTarget& a, const ScoredTarget& b) {
                    return a.score > b.score; // Giảm dần: tốt nhất lên đầu
                });

            // Loại bỏ từ vị trí maxCount trở đi
            for (int idx = maxCount; idx < (int)assigned.size(); ++idx)
                sol.at(i, assigned[idx].j) = 0;
        }
    }
    // Xử lý theo thứ tự ưu tiên: mục tiêu quan trọng (Priority nhỏ) trước
    std::vector<int> targetOrder(m);
    std::iota(targetOrder.begin(), targetOrder.end(), 0);
    std::sort(targetOrder.begin(), targetOrder.end(), [&](int a, int b) {
        return prob_.targets[a].priority < prob_.targets[b].priority;
        });

    for (int idx = 0; idx < m; ++idx)
    {
        int    j = targetOrder[idx];
        double Ej = prob_.targets[j].explosive_required;
        if (Ej <= 0.0) continue;

        // Tính tổng lượng nổ hiện tại và danh sách UAV đã gán
        double totalExplosive = 0.0;
        std::vector<int> assignedList;
        for (int i = 0; i < n; ++i)
        {
            if (sol.at(i, j) != 1) continue;
            totalExplosive += prob_.uavs[i].explosive;
            assignedList.push_back(i);
        }
        /////////////
        // ── Trường hợp DƯ: loại UAV explosive nhỏ nhất nếu còn đủ ──
        if (totalExplosive > Ej && !assignedList.empty())
        {
            // Sắp xếp theo explosive tăng dần: thử loại nhỏ nhất trước
            std::sort(assignedList.begin(), assignedList.end(), [&](int a, int b) {
                return prob_.uavs[a].explosive < prob_.uavs[b].explosive;
                });
            for (int iRemove : assignedList)
            {
                double e = prob_.uavs[iRemove].explosive;
                if (totalExplosive - e >= Ej)
                {
                    // Vẫn đủ sau khi loại → giải phóng UAV này
                    sol.at(iRemove, j) = 0;
                    totalExplosive -= e;
                }
                // Nếu không thể loại thêm → dừng
            }
            continue; // Sang mục tiêu tiếp theo
        }
        // ── Trường hợp THIẾU: tìm UAV bổ sung ──
        if (totalExplosive < Ej)
        {
            double deficit = Ej - totalExplosive; // Lượng nổ còn thiếu

            // Tìm các UAV ứng viên bổ sung:
            // điều kiện: chưa gán j, a_{ij}=1, còn slot, có explosive > 0
            struct Candidate {
                int    i;
                double explosive;
                double gap; // |explosive - deficit|: nhỏ hơn = phù hợp hơn
            };
            std::vector<Candidate> candidates;

            for (int i = 0; i < n; ++i)
            {
                if (sol.at(i, j) == 1) continue;             // Đã gán rồi
                if (prob_.uavs[i].aij[j] == 0) continue;     // Không khả dụng
                if (prob_.uavs[i].explosive <= 0.0) continue; // Trinh sát, bỏ qua

                // Kiểm tra còn slot (chưa đạt maxCount)
                int usedSlots = 0;
                for (int jj = 0; jj < m; ++jj)
                    usedSlots += sol.at(i, jj);
                if (usedSlots >= prob_.uavs[i].maxCount) continue;

                double ei = prob_.uavs[i].explosive;
                double gap = std::abs(ei - deficit); // Gần deficit = ưu tiên
                candidates.push_back({ i, ei, gap });
            }

            std::uniform_real_distribution<double> jitter(-0.5, 0.5);
            for (auto& cand : candidates)
                cand.gap += jitter(rng());

            std::sort(candidates.begin(), candidates.end(),
                [](const Candidate& a, const Candidate& b) { return a.gap < b.gap; });

            // Thêm từng UAV bổ sung cho đến khi đủ lượng nổ
            for (auto& cand : candidates)
            {
                if (totalExplosive >= Ej) break;
                sol.at(cand.i, j) = 1;
                totalExplosive += cand.explosive;
                deficit -= cand.explosive;
            }

            // Sau khi cố tìm mà vẫn không đủ → hủy toàn bộ phân công j
            // (phân công nửa vời không tiêu diệt được mục tiêu → lãng phí UAV)
            if (totalExplosive < Ej)
            {
                for (int i = 0; i < n; ++i)
                    sol.at(i, j) = 0;
            }
        }

    }
    // Phòng ngừa crossover/mutate vô tình gán x_{ij}=1 cho cặp không khả dụng
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            if (prob_.uavs[i].aij[j] == 0)
                sol.at(i, j) = 0;
}


// Chạy GA
AssignmentSolution UAVGAOptimizer::run()
{
    std::cout << "[GA] Seed = " << std::random_device{}() << "\n";
    initPopulation();

    AssignmentSolution best = population_[0];
    for (auto& sol : population_)
        if (sol.fitness > best.fitness) best = sol;
    for (int gen = 0; gen < maxGen_; ++gen)
    {
        std::vector<AssignmentSolution> newPop;
        newPop.reserve(popSize_);
        newPop.push_back(best);
        for (int k = 0; k < popSize_; ++k)
        {
            AssignmentSolution p1 = selectParent();
            AssignmentSolution p2 = selectParent();
            AssignmentSolution child = crossover(p1, p2);
            mutate(child);
            repair(child);
            evaluate(child);
            newPop.push_back(child);

            if (child.fitness > best.fitness)
                best = child;
        }

        population_ = std::move(newPop);
    }

    return best;
}



OptimizationProblem OptimizationBuilder::build(const UnitUAVList& unitList,
    const Graph& graph, const std::string& dataDir)
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
        to.value = t.value;
        to.x = t.x;
        to.y = t.y;
        to.vertexId = t.id_vertex;
        to.explosive_required = t.explosive;
        to.priority = t.priority; // Đọc thẳng từ CSV (cột K)
        prob.targets.push_back(to);         // Chỉ push 1 lần

        std::cout << "[BUILD] Target " << to.id
            << " \"" << to.name << "\""
            << " | vertex=" << to.vertexId
            << " | E_j=" << to.explosive_required
            << " | v_j=" << to.value
            << " | priority=" << to.priority << "\n";
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
            opt.ValuePerAttack = u.getCost();
            opt.maxCount = (u.getQuantity() >= 1) ? u.getQuantity() : 1; 
            opt.unitIndex = unitList.getUnitIndex(unit.getUnitId());
            opt.unitName = unit.getUnitName();
            opt.aij.resize(m, 1);   // Mặc định khả dụng với mọi mục tiêu
            opt.pij.resize(m, 0.0); // Sẽ load từ Probability.csv

            // ── RANGE CHECK: Tính khoảng cách Dijkstra thực tế ──
            // Tìm đỉnh xuất phát của đơn vị 
            int startV = unit.getVertexId();
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
                << " | value=" << opt.ValuePerAttack << "\n";
        }
    }

    int n = (int)prob.uavs.size();
    std::cout << "[BUILD] Tong: " << n << " UAV tan cong, " << m << " muc tieu\n";

    // PHẦN 3: GÁN XÁC SUẤT p_{ij} TỪ Probability.csv
    auto pijMap = loadPij(dataDir + "\\Probability.csv");

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
     // Với mỗi UAV tấn công i được gán mục tiêu j:
     //   path = shortestPath(startV_đơn_vị → vertexId_mục_tiêu)
    best.paths.resize(n, std::vector<std::vector<int>>(m));

    for (int i = 0; i < n; ++i)
    {
        const UAVTypeOpt& uav = prob.uavs[i];
        const UnitUAV& unit = unitList.getUnit(uav.unitIndex);

        // Đỉnh xuất phát: đỉnh gần nhất với tọa độ căn cứ đơn vị.
        int startV = unit.getVertexId();

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

