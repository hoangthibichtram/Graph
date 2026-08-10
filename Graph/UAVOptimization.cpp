#include "UAVOptimization.h"
#include "OptimizationProblem.h"
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric> 
#include <chrono>

static std::mt19937& rng()
{
    static std::mt19937 gen(
        std::random_device{}() ^
        (uint32_t)std::chrono::steady_clock::now().time_since_epoch().count()
    );
    return gen;
}

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
    return prob_.targets[a].Priority < prob_.targets[b].Priority;
    });
    population_.clear();
    population_.resize(popSize_);

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
            // ── Khởi tạo "định hướng": gán UAV theo mục tiêu ưu tiên cao ──
            // Duyệt từng mục tiêu theo thứ tự ưu tiên,
            // cố gắng tìm UAV phù hợp (a_{ij}=1) chưa được gán
            for (int jIdx = 0; jIdx < m; ++jIdx)
            {
                int j = targetByPriority[jIdx];
                double accumulated = 0.0;
                for (int i = 0; i < n; ++i)
                {
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
// Hàm thích nghi F = ∑ v_j (1 - ∏ (1 - p_ij)^{x_ij}).
void UAVGAOptimizer::evaluate(AssignmentSolution& sol)
{
    if ((int)sol.x.size() != sol.nUavTypes * sol.nTargets) return;

    int n = sol.nUavTypes;
    int m = sol.nTargets;

    // ── Bước 1: Tính S_j và kill probability cho từng mục tiêu ──
    double F = 0.0;
    for (int j = 0; j < m; ++j)
    {
        // S_j = ∏_i (1 - p_{ij})  với mọi i có x_{ij}=1
        double Sj = 1.0;
        for (int i = 0; i < n; ++i)
        {
            if (sol.at(i, j) == 1)
                Sj *= (1.0 - prob_.uavs[i].pij[j]);
        }
        // Đóng góp vào hàm mục tiêu: v_j * (1 - S_j)
        F += prob_.targets[j].value * (1.0 - Sj);
    }

    // 3. Penalty cho ràng buộc explosive (thiếu hụt)
    const double k = 2.0;
    double penalty = 0.0;

    for (int j = 0; j < m; ++j)
    {
        double required = prob_.targets[j].explosive_required;
        if (required <= 0) continue;

        bool anyAssigned = false;
        double totalExplosive = 0.0;
        for (int i = 0; i < n; ++i)
        {
            if (sol.at(i, j) == 1)           
            {
                totalExplosive += prob_.uavs[i].explosive;
                anyAssigned = true;
            }
        }
        if (anyAssigned) {
            double shortage = std::max(0.0, required - totalExplosive);
            double shortageRatio = shortage / required; // ∈ [0,1]
            double vj = prob_.targets[j].value;
            penalty += shortageRatio * vj * k;
        }
    }

    sol.fitness = F - penalty;
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

// Lai ghép 1 điểm (giữ nguyên x, nhưng nhớ copy uavs)
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

// Đột biến: tăng/giảm số UAV tại (i,j)
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

    // ========== 1. Ràng buộc số lượng & ngân sách cho từng UAV ==========
    for (int i = 0; i < n; ++i)
    {

        const auto& u = prob_.uavs[i];
        int maxCount = u.maxCount;      // = 1 trong bài toán này
        double cost = u.costPerAttack;

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
            double eff = (cost > 0.0) ? (vj * pij / cost) : (vj * pij);

            int    prio = (prob_.targets[j].Priority > 0) ? prob_.targets[j].Priority : 1;
            double priorityW = 1.0 / (double)prio;
            double combined = eff * priorityW;

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
        return prob_.targets[a].Priority < prob_.targets[b].Priority;
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
