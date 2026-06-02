#include "UAVOptimization.h"
#include "OptimizationProblem.h"
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric> 

static std::mt19937& rng()
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
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
        population_[0].nUavTypes = n;
        population_[0].nTargets = m;
        population_[0].x.clear();
        population_[0].fitness = 0.0;
        return;
    }

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

        // Khởi tạo: phân bổ ngẫu nhiên ngẫu nhiên xij = 0 hoặc 1
        for (int i = 0; i < n; ++i)
        {
            // 95% UAV được giao nhiệm vụ (có thể điều chỉnh)
            if (probDist(rng()) < 0.95)
            {
                int j = targetDist(rng());  // chọn ngẫu nhiên một mục tiêu
                if (j < (int)prob_.uavs[i].aij.size() && prob_.uavs[i].aij[j] != 0)
                    sol.at(i, j) = 1;
            }
        }

        evaluate(sol);
        population_[k] = sol;
    }
}
// Hàm thích nghi F = ∑ v_j (1 - ∏ (1 - p_ij)^{x_ij}).
void UAVGAOptimizer::evaluate(AssignmentSolution& sol)
{
    if (sol.x.size() != sol.nUavTypes * sol.nTargets)
        return;

    int n = sol.nUavTypes;
    int m = sol.nTargets;

    // 1. Tính xác suất tiêu diệt từng mục tiêu
    std::vector<double> killProb(m, 0.0);
    for (int j = 0; j < m; ++j)
    {
        double miss = 1.0;
        for (int i = 0; i < n; ++i)
        {
            if (sol.at(i, j) == 1 && j < (int)prob_.uavs[i].pij.size())  
                miss *= (1.0 - prob_.uavs[i].pij[j]);
        }
        killProb[j] = 1.0 - miss;
    }

    // 2. Tổng giá trị kỳ vọng
    double expectedValue = 0.0;
    for (int j = 0; j < m; ++j)
        expectedValue += prob_.targets[j].value * killProb[j];

    // 3. Penalty cho ràng buộc explosive (thiếu hụt)
    double penalty = 0.0;
    const double PENALTY_FACTOR = 10000.0;
    for (int j = 0; j < m; ++j)
    {
        double required = prob_.targets[j].explosive_required;
        if (required <= 0) continue;

        double totalExplosive = 0.0;
        for (int i = 0; i < n; ++i)
        {
            if (sol.at(i, j) == 1)           
                totalExplosive += prob_.uavs[i].explosive;
        }

        double shortage = std::max(0.0, required - totalExplosive);
        double surplus = std::max(0.0, totalExplosive - required);
        penalty += shortage * 10000.0 + surplus * 10.0;
    }

    sol.fitness = expectedValue - penalty;
}

// Chọn lọc roulette
AssignmentSolution UAVGAOptimizer::selectParent()
{
    double sumFit = 0.0;
    for (auto& s : population_) sumFit += s.fitness;
    if (sumFit <= 0.0) return population_[0];

    std::uniform_real_distribution<double> dist(0.0, sumFit);
    double r = dist(rng());

    double acc = 0.0;
    for (auto& s : population_)
    {
        acc += s.fitness;
        if (acc >= r) return s;
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
            // Nếu UAV không thể tấn công mục tiêu j → luôn 0
            if (prob_.uavs[i].aij[j] == 0)
            {
                child.at(i, j) = 0;
                continue;
            }

            // Đột biến: đảo bit 0 ↔ 1
            if (prob(rng()) < pm_)
            {
                int& xij = child.at(i, j);
                xij = 1 - xij;
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
    if (m <= 0) return;

    // ========== 1. Ràng buộc số lượng & ngân sách cho từng UAV ==========
    for (int i = 0; i < n; ++i)
    {
        const auto& u = prob_.uavs[i];
        int maxCount = u.maxCount;
        double maxBudget = u.maxBudget;
        double cost = u.costPerAttack;

        // Thu thập các mục tiêu mà UAV i đang tấn công
        struct Item
        {
            int j;
            double efficiency;   // value * p_ij / cost
            double priorityWeight;
        };
        std::vector<Item> items;
        int totalCount = 0;
        double totalCost = 0.0;

        for (int j = 0; j < m; ++j)
        {
            if (sol.at(i, j) == 1)
            {
                totalCount++;
                totalCost += cost;

                double vj = prob_.targets[j].value;
                double pij = prob_.uavs[i].pij[j];
                double eff = (cost > 0) ? (vj * pij) / cost : (vj * pij);
                double priorityWeight = (prob_.targets[j].Priority > 0)
                    ? 1.0 / prob_.targets[j].Priority
                    : 1.0;// Priority càng nhỏ càng quan trọng

                items.push_back({ j, eff, priorityWeight });
            }
        }

        // Nếu vi phạm, cần loại bỏ các mục tiêu kém hiệu quả nhất
        if (totalCount > maxCount || totalCost > maxBudget)
        {
            // Tính combined score = efficiency * priorityWeight
            std::vector<std::pair<int, double>> scored;
            for (const auto& it : items)
            {
                double combined = it.efficiency * it.priorityWeight;
                scored.push_back({ it.j, combined });
            }

            // Sắp xếp tăng dần (thấp nhất lên đầu)
            std::sort(scored.begin(), scored.end(),
                [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                    return a.second < b.second;
                });

            // Loại bỏ từ đầu danh sách cho đến khi thỏa mãn
            while ((totalCount > maxCount || totalCost > maxBudget) && !scored.empty())
            {
                int jRemove = scored.front().first;
                sol.at(i, jRemove) = 0;
                totalCount--;
                totalCost -= cost;
                scored.erase(scored.begin());

                // Cũng xóa khỏi items nếu cần (không bắt buộc)
                auto it = std::find_if(items.begin(), items.end(),
                    [jRemove](const Item& itm) { return itm.j == jRemove; });
                if (it != items.end()) items.erase(it);
            }
        }
    }

    // ============================================================
    // PHẦN 2: Ràng buộc lượng nổ (explosive) cho từng mục tiêu 
    // ============================================================
    for (int j = 0; j < m; ++j)
    {
        double required = prob_.targets[j].explosive_required;
        if (required <= 0) continue;

        double totalExplosive = 0.0;
        struct UavInfo
        {
            int i;
            double eff;
            double explosive;
            double priorityWeight;
        };
        std::vector<UavInfo> assigned;

        for (int i = 0; i < n; ++i)
        {
            if (sol.at(i, j) == 1)
            {
                double vj = prob_.targets[j].value;
                double pij = prob_.uavs[i].pij[j];
                double cost = prob_.uavs[i].costPerAttack;
                double eff = (cost > 0) ? (vj * pij) / cost : (vj * pij);
                double priorityWeight = 1.0 / prob_.targets[j].Priority;
                assigned.push_back({ i, eff, prob_.uavs[i].explosive, priorityWeight });
                totalExplosive += prob_.uavs[i].explosive;
            }
        }

        // Nếu tổng explosive vượt quá yêu cầu, cần loại bỏ bớt UAV kém hiệu quả
        if (totalExplosive > required)
        {
            // Tính combined score cho từng UAV (eff * priorityWeight)
            std::vector<std::pair<int, double>> scored;
            for (const auto& u : assigned)
            {
                double combined = u.eff * u.priorityWeight;
                scored.push_back({ u.i, combined });
            }

            // Sắp xếp tăng dần (hiệu quả thấp nhất lên đầu)
            std::sort(scored.begin(), scored.end(),
                [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                    return a.second < b.second;
                });

            // Loại bỏ UAV có combined score thấp nhất
            for (const auto& candidate : scored)
            {
                int iRemove = candidate.first;
                double explosiveRemove = 0.0;
                for (const auto& u : assigned)
                    if (u.i == iRemove) { explosiveRemove = u.explosive; break; }
                if (totalExplosive - explosiveRemove >= required)
                {
                    sol.at(iRemove, j) = 0;
                    totalExplosive -= explosiveRemove;
                    // Cập nhật lại danh sách assigned nếu cần (không bắt buộc)
                }
                else
                {
                    // Nếu không thể loại bỏ thêm nữa thì dừng
                    break;
                }
            }
        }
    }
}


// Chạy GA
AssignmentSolution UAVGAOptimizer::run()
{
    initPopulation();

    AssignmentSolution best = population_[0];

    for (int gen = 0; gen < maxGen_; ++gen)
    {
        std::vector<AssignmentSolution> newPop;
        newPop.reserve(popSize_);

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
