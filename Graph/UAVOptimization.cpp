#include "UAVOptimization.h"
#include "OptimizationProblem.h"
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>

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

    std::uniform_int_distribution<int> bit(0, 1);

    for (int k = 0; k < popSize_; ++k)
    {
        AssignmentSolution sol;
        sol.nUavTypes = n;
        sol.nTargets = m;
        sol.x.assign(n * m, 0);

        // Khởi tạo: phân bổ ngẫu nhiên ngẫu nhiên xij = 0 hoặc 1
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < m; ++j)
            {
                if (prob_.uavs[i].aij[j] == 0)
                    sol.at(i, j) = 0;
                else
                    sol.at(i, j) = bit(rng());   // Chỉ sử dụng 0 hoặc 1 như nguyên thủy
            }
        }

        repair(sol);
        evaluate(sol);
        population_[k] = sol;
    }
}

// Hàm thích nghi F = ∑ v_j (1 - ∏ (1 - p_ij)^{x_ij}) - CÓ KHẤU TRỪ PHÒNG KHÔNG CHI VIỆN
void UAVGAOptimizer::evaluate(AssignmentSolution& sol)
{
    if (sol.x.size() != sol.nUavTypes * sol.nTargets)
        return;

    int n = sol.nUavTypes;
    int m = sol.nTargets;

    std::vector<double> F_killProb(m, 0.0);
    std::vector<int> activeDefenses;

    // 1. Tính toán hiệu quả rải bom thô & Nhận diện Trạm Phòng Không còn sống
    for (int j = 0; j < m; ++j)
    {
        double Sj = 1.0;
        for (int i = 0; i < n; ++i)
        {
            int xij = sol.at(i, j);
            if (xij <= 0) continue;

            double missOne = 1.0 - prob_.uavs[i].pij[j];
            double missAll = std::pow(missOne, xij);
            Sj *= missAll;
        }

        double killProb = 1.0 - Sj;
        F_killProb[j] = killProb;
    }

    // 2. Chấm điểm Thích nghi nguyên thủy (Chỉ tính sát thương)
    double F = 0.0;
    for (int j = 0; j < m; ++j)
    {
        double hitRate = F_killProb[j];
        double vj = prob_.targets[j].value;
        F += vj * hitRate; // Điểm thu về  từ sát thương triệt để
    }

    sol.fitness = F;
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
    {
        sol.x.assign(sol.nUavTypes * sol.nTargets, 0);
    }

    int n = sol.nUavTypes;   // số lượng UAV
    int m = sol.nTargets;    // số mục tiêu
    if (m <= 0) return;

    for (int i = 0; i < n; ++i)
    {
        const auto& u = prob_.uavs[i];
        int maxCount = u.maxCount;
        double maxBudget = u.maxBudget;
        double cost = u.costPerAttack;

        // Tính hiệu suất eij = vj * pij / cost
        struct Item { int j; double e; };
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
                
                // Tránh lỗi chia cho 0 nếu cost = 0, nếu cost bằng 0 thì ta cho hiệu suất cao (ví dụ vj * pij)
                double e = (cost > 0 ? (vj * pij) / cost : (vj * pij));

                items.push_back({ j, e });//
            }
        }

        // Nếu vi phạm ràng buộc → loại bỏ mục tiêu có hiệu suất thấp nhất
        while (totalCount > maxCount || totalCost > maxBudget)
        {
            if (items.empty()) break;

            // sắp xếp tăng dần theo hiệu suất
            std::sort(items.begin(), items.end(),
                [](const Item& a, const Item& b) { return a.e < b.e; });

            int jRemove = items.front().j;
            sol.at(i, jRemove) = 0; // Trực tiếp đặt thành 0 để chặn đánh mục tiêu này

            totalCount--;
            totalCost -= cost;

            items.erase(items.begin());
        }
    }

    // 2. Ràng buộc lượng nổ cho từng mục tiêu
    for (int j = 0; j < m; ++j)
    {
        double requiredExplosive = prob_.targets[j].explosive_required; // field này phải có trong target
        double totalExplosive = 0.0;
        struct UavAssign { int i; double eff; double explosive; double cost; };
        std::vector<UavAssign> assigned;

        for (int i = 0; i < n; ++i)
        {
            if (sol.at(i, j) == 1)
            {
                double vj = prob_.targets[j].value;
                double pij = prob_.uavs[i].pij[j];
                double cost = prob_.uavs[i].costPerAttack;
                double explosive = prob_.uavs[i].explosive; // field này phải có trong UAV
                double eff = (cost > 0 ? (vj * pij) / cost : (vj * pij));
                assigned.push_back({ i, eff, explosive, cost });
                totalExplosive += explosive;
            }
        }

        // Nếu tổng lượng nổ vượt yêu cầu, loại bỏ UAV hiệu suất thấp cho đến khi vừa đủ
        while (!assigned.empty() && totalExplosive - assigned.front().explosive >= requiredExplosive)
        {
            std::sort(assigned.begin(), assigned.end(),
                [](const UavAssign& a, const UavAssign& b) { return a.explosive < b.explosive; });
            int iRemove = assigned.front().i;
            totalExplosive -= assigned.front().explosive;
            sol.at(iRemove, j) = 0;
            assigned.erase(assigned.begin());
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
