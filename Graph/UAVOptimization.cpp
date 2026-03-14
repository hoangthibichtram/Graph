#include "UAVOptimization.h"
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
        sol.uavs = prob_.uavs; // copy thông tin UAV

        // Khởi tạo: phân bổ ngẫu nhiên nhưng không vượt quá maxCount
        for (int i = 0; i < n; ++i)
        {
            int remaining = sol.uavs[i].maxCount; // số UAV loại i còn lại

            for (int j = 0; j < m; ++j)
            {
                if (remaining <= 0)
                {
                    sol.at(i, j) = 0;
                    continue;
                }

                // Nếu UAV loại i không thể tấn công mục tiêu j
                if (prob_.uavs[i].aij[j] == 0)
                {
                    sol.at(i, j) = 0;
                    continue;
                }

                std::uniform_int_distribution<int> dist(0, remaining);
                int use = dist(rng()); // dùng 0..remaining UAV
                sol.at(i, j) = use;
                remaining -= use;
            }
        }

        repair(sol);
        evaluate(sol);
        population_[k] = sol;

        std::cout << "Init sol: x.size=" << sol.x.size()
            << " expected=" << n * m
            << "  n=" << n << " m=" << m << "\n";
    }
}

// Hàm thích nghi F = ∑ v_j (1 - ∏ (1 - p_ij)^{x_ij})
void UAVGAOptimizer::evaluate(AssignmentSolution& sol)
{
    if (sol.x.size() != sol.nUavTypes * sol.nTargets)
        return;

    int n = sol.nUavTypes; // số loại UAV
    int m = sol.nTargets;  // số mục tiêu

    double F = 0.0;

    for (int j = 0; j < m; ++j) // mục tiêu j
    {
        double Sj = 1.0; // xác suất mục tiêu j sống sót

        for (int i = 0; i < n; ++i) // UAV loại i
        {
            int xij = sol.at(i, j);
            if (xij <= 0) continue;

            double pij = prob_.uavs[i].pij[j]; // xác suất 1 UAV loại i đánh trúng mục tiêu j
            double missOne = 1.0 - pij;

            // (1 - pij)^{xij}
            double missAll = std::pow(missOne, xij);
            Sj *= missAll;
        }

        double vj = prob_.targets[j].value; // độ quan trọng / giá trị mục tiêu j
        F += vj * (1.0 - Sj);               // vj * P_bi_tieu_diet
    }

    sol.fitness = F; // GA đang tối đa hóa F
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
    child.uavs = p1.uavs;
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
            if (prob_.uavs[i].aij[j] == 0)
            {
                child.at(i, j) = 0;
                continue;
            }

            if (prob(rng()) < pm_)
            {
                int& xij = child.at(i, j);
                // delta = -1, 0, +1
                std::uniform_int_distribution<int> d(-1, 1);
                int delta = d(rng());
                xij = std::max(0, xij + delta);
            }
        }
    }
}

// Sửa ràng buộc: không vượt quá số UAV loại i (∑_j x_ij ≤ maxCount_i)
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

    // Với mỗi loại UAV i
    for (int i = 0; i < n; ++i)
    {
        auto& u = sol.uavs[i];
        int maxCount = u.maxCount;

        while (true)
        {
            int totalCount = 0;

            // ⭐ Bước 1: đảm bảo không có xij âm
            for (int j = 0; j < m; ++j)
            {
                if (sol.at(i, j) < 0)
                    sol.at(i, j) = 0;

                totalCount += sol.at(i, j);
            }

            // ⭐ Bước 2: nếu tổng số UAV loại i không vượt quá maxCount → OK
            if (totalCount <= maxCount)
                break;

            // ⭐ Bước 3: nếu vượt → giảm ngẫu nhiên 1 mục tiêu
            int j = distTarget(rng());
            if (sol.at(i, j) > 0)
            {
                sol.at(i, j)--;
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
