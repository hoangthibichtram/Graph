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
    int n = (int)prob_.uavs.size();    // số loại UAV
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

    for (int k = 0; k < popSize_; ++k)
    {
        AssignmentSolution sol;
        sol.nUavTypes = n;
        sol.nTargets = m;
        sol.x.assign(n * m, 0);
        // copy thông tin UAV

        // Khởi tạo: phân bổ ngẫu nhiên nhưng không vượt quá maxCount
        for (int i = 0; i < n; ++i)
        {
            std::uniform_int_distribution<int> bit(0, 1);

            for (int i = 0; i < n; ++i)
            {
                for (int j = 0; j < m; ++j)
                {
                    if (prob_.uavs[i].aij[j] == 0)
                        sol.at(i, j) = 0;
                    else
                        sol.at(i, j) = bit(rng());   // xij = 0 hoặc 1
                }
            }

                
        }

        repair(sol);
        evaluate(sol);
        population_[k] = sol;
    }
}

// Hàm thích nghi F = ∑ v_j (1 - ∏ (1 - p_ij)^{x_ij})
void UAVGAOptimizer::evaluate(AssignmentSolution& sol)
{
    if (sol.x.size() != sol.nUavTypes * sol.nTargets)
        return;

    int n = sol.nUavTypes;
    int m = sol.nTargets;

    double F = 0.0;

    for (int j = 0; j < m; ++j)
    {
        double Sj = 1.0;

        for (int i = 0; i < n; ++i)
        {
            int xij = sol.at(i, j);
            if (xij <= 0) continue;

            double pij = prob_.uavs[i].pij[j];
           /* std::cout << "i=" << i << " j=" << j
                << " xij=" << xij
                << " pij=" << pij << "\n";*/

            double missOne = 1.0 - pij;
            double missAll = std::pow(missOne, xij);
            Sj *= missAll;
        }

        double vj = prob_.targets[j].value;
       // std::cout << "j=" << j << " vj=" << vj << "\n";

        F += vj * (1.0 - Sj);
    }

   // std::cout << "F = " << F << "\n";
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

    int n = sol.nUavTypes;   // số loại UAV
    int m = sol.nTargets;    // số mục tiêu
    if (m <= 0) return;

    std::uniform_int_distribution<int> distTarget(0, m - 1);

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
                double e = (cost > 0 ? (vj * pij) / cost : 0.0);

                items.push_back({ j, e });
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
            sol.at(i, jRemove) = 0;

            totalCount--;
            totalCost -= cost;

            items.erase(items.begin());
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
