#include "Optimization.h"
#include "ExactSolver.h"
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
#include <iomanip>

// ===========================================================================
//  HAM PHU TRO
// ===========================================================================
static uint32_t g_seed = 0u;
static bool     g_gaVerbose = true;   // chi in tien trinh khi chay mot lan

static std::mt19937& rng()
{
    static std::mt19937 gen = [] {
        g_seed = (uint32_t)std::random_device{}() ^
                 (uint32_t)std::chrono::steady_clock::now().time_since_epoch().count();
        return std::mt19937(g_seed);
    }();
    return gen;
}
static uint32_t currentSeed() { return g_seed; }

// Gieo lai hat giong de ket qua tai lap duoc giua cac lan chay.
static void seedRng(uint32_t s)
{
    rng().seed(s);   // ep khoi tao bo sinh TRUOC, tranh bi ghi de g_seed
    g_seed = s;      // gan SAU nen con so ghi lai dung voi hat giong thuc te
}

// Doc bang xac suat p_ij. Khoa tra cuu la MA CHUNG LOAI, khong phai ma ca the.
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
    std::getline(ifs, line); // bo qua header
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

// Doc tham so chien dich tu Config.csv (dinh dang: key,value)
//   campaign_budget : C  (USD)
//   cost_coef_k     : k  (USD/km)
// Chep nguyen ven mot tep (dung cho buoc dong bo sang du an ba chieu).
static bool copyFileBinary(const std::string& from, const std::string& to)
{
    std::ifstream in(from, std::ios::binary);
    if (!in) return false;
    std::ofstream out(to, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << in.rdbuf();
    return out.good();
}

static void loadConfig(const std::string& path, OptimizationProblem& prob)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cout << "[CFG] Khong tim thay Config.csv -> dung mac dinh"
                  << " (C khong gioi han, k=" << prob.costCoefK << ")\n";
        return;
    }
    auto trim = [](std::string& s) {
        s.erase(0, s.find_first_not_of(" \t\r\n"));
        s.erase(s.find_last_not_of(" \t\r\n") + 1);
        };
    std::string line;
    while (std::getline(ifs, line))
    {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string key, val;
        std::getline(ss, key, ',');
        std::getline(ss, val, ',');
        trim(key); trim(val);
        if (val.empty()) continue;
        try {
            if (key == "campaign_budget") prob.campaignBudget = std::stod(val);
            else if (key == "cost_coef_k") prob.costCoefK = std::stod(val);
            else if (key == "ue_data_dir") prob.ueDataDir = val;
            else if (key == "ga_seed")     prob.gaSeed = (unsigned)std::stoul(val);
            else if (key == "ga_runs")     prob.gaRuns = std::stoi(val);
            else if (key == "run_exact")   prob.runExact = (std::stoi(val) != 0);
        }
        catch (...) {}
    }
    std::cout << "[CFG] C = " << prob.campaignBudget
              << " USD | k = " << prob.costCoefK << " USD/km\n";
    if (!prob.ueDataDir.empty())
        std::cout << "[CFG] Thu muc du an 3D: " << prob.ueDataDir << "\n";
    if (prob.gaSeed != 0u)
        std::cout << "[CFG] Hat giong co dinh ga_seed = " << prob.gaSeed << "\n";
    if (prob.gaRuns > 1)
        std::cout << "[CFG] Che do danh gia: chay lap " << prob.gaRuns << " lan\n";
}

// ===========================================================================
//  THUAT TOAN DI TRUYEN
// ===========================================================================
UAVGAOptimizer::UAVGAOptimizer(const OptimizationProblem& problem,
    int populationSize, int maxGenerations,
    double crossoverRate, double mutationRate)
    : prob_(problem), popSize_(populationSize), maxGen_(maxGenerations)
    , pc_(crossoverRate), pm_(mutationRate)
{
}

// Kiem tra UAV i da nhan nhiem vu nao chua - hien thuc rang buoc (2.7)
static inline bool isBusy(const AssignmentSolution& sol, int i, int m)
{
    for (int j = 0; j < m; ++j)
        if (sol.at(i, j) == 1) return true;
    return false;
}

void UAVGAOptimizer::initPopulation()
{
    int n = (int)prob_.uavs.size();     // so UAV CA THE
    int m = (int)prob_.targets.size();

    if (n <= 0 || m <= 0) {
        population_.assign(1, { n, m, std::vector<int>(n * m, 0), 0.0 });
        return;
    }

    population_.clear();
    population_.resize(popSize_);

    // Thu tu muc tieu theo do uu tien (priority = 1 la quan trong nhat)
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
            // --- Khoi tao DINH HUONG: uu tien muc tieu quan trong truoc ---
            std::vector<int> uavOrder(n);
            std::iota(uavOrder.begin(), uavOrder.end(), 0);
            for (int s = n - 1; s > 0; --s) {
                std::uniform_int_distribution<int> pick(0, s);
                std::swap(uavOrder[s], uavOrder[pick(rng())]);
            }
            for (int jIdx = 0; jIdx < m; ++jIdx)
            {
                int j = targetByPriority[jIdx];
                double accumulated = 0.0;
                for (int o = 0; o < n; ++o)
                {
                    int i = uavOrder[o];
                    if (prob_.uavs[i].aij[j] == 0) continue;   // (2.10)
                    if (isBusy(sol, i, m))         continue;   // (2.7)
                    sol.at(i, j) = 1;
                    accumulated += prob_.uavs[i].explosive;
                    if (accumulated >= prob_.targets[j].explosive_required) break;
                }
            }
        }
        else
        {
            // --- Khoi tao NGAU NHIEN: moi UAV chon toi da 1 muc tieu ---
            for (int i = 0; i < n; ++i)
            {
                if (probDist(rng()) >= 0.9) continue;  // 10% de UAV "nghi" -> da dang gen
                int j = targetDist(rng());
                if (prob_.uavs[i].aij[j] != 0) sol.at(i, j) = 1;
            }
        }
        repair(sol);
        evaluate(sol);
        population_[k] = sol;
    }
}

// Chon loc banh xe roulette tren do thich nghi da tinh tien ve mien duong
AssignmentSolution UAVGAOptimizer::selectParent()
{
    double minFit = population_[0].fitness;
    for (auto& s : population_)
        if (s.fitness < minFit) minFit = s.fitness;

    const double eps = 1e-9;
    double sumShifted = 0.0;
    std::vector<double> shifted(population_.size());
    for (int k = 0; k < (int)population_.size(); ++k) {
        shifted[k] = population_[k].fitness - minFit + eps;
        sumShifted += shifted[k];
    }
    std::uniform_real_distribution<double> dist(0.0, sumShifted);
    double r = dist(rng()), acc = 0.0;
    for (int k = 0; k < (int)population_.size(); ++k) {
        acc += shifted[k];
        if (acc >= r) return population_[k];
    }
    return population_.back();
}

AssignmentSolution UAVGAOptimizer::crossover(const AssignmentSolution& p1,
                                             const AssignmentSolution& p2)
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
    if (prob(rng()) < pc_) {
        int cut = posDist(rng());
        for (int k = cut; k < L; ++k) child.x[k] = p2.x[k];
    }
    return child;
}

void UAVGAOptimizer::mutate(AssignmentSolution& child)
{
    int n = child.nUavTypes, m = child.nTargets;
    std::uniform_real_distribution<double> prob(0.0, 1.0);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
        {
            if (prob_.uavs[i].aij[j] == 0) { child.at(i, j) = 0; continue; }
            if (prob(rng()) < pm_) child.at(i, j) = 1 - child.at(i, j);
        }
}

// ---------------------------------------------------------------------------
//  DANH GIA: F = SUM_j v_j * (1 - PROD_i (1 - p_ij)^x_ij)
//  Phuong an vi pham bat ky rang buoc nao deu bi loai bang do thich nghi rat thap.
//  Khong con so hang phat: toan tu sua chua da bao dam tinh kha thi.
// ---------------------------------------------------------------------------
void UAVGAOptimizer::evaluate(AssignmentSolution& sol)
{
    const double INFEASIBLE = -1e9;
    int n = sol.nUavTypes, m = sol.nTargets;

    double cost = 0.0;
    for (int i = 0; i < n; ++i)
    {
        int cnt = 0;
        for (int j = 0; j < m; ++j)
        {
            if (sol.at(i, j) != 1) continue;
            if (prob_.uavs[i].aij[j] == 0) { sol.fitness = INFEASIBLE; return; } // (2.10)
            ++cnt;
            cost += prob_.uavs[i].cij[j];
        }
        if (cnt > 1) { sol.fitness = INFEASIBLE; return; }                       // (2.7)
    }
    if (prob_.campaignBudget > 0.0 && cost > prob_.campaignBudget + 1e-9)
    {
        sol.fitness = INFEASIBLE; return;                                        // (2.6)
    }

    double F = 0.0;
    for (int j = 0; j < m; ++j)
    {
        double Sj = 1.0, totalExplosive = 0.0;
        bool anyAssigned = false;
        for (int i = 0; i < n; ++i)
        {
            if (sol.at(i, j) != 1) continue;
            Sj *= (1.0 - prob_.uavs[i].pij[j]);
            totalExplosive += prob_.uavs[i].explosive;
            anyAssigned = true;
        }
        if (!anyAssigned) continue;                       // y_j = 0 : hop le, khong tinh diem
        if (totalExplosive < prob_.targets[j].explosive_required)
        {
            sol.fitness = INFEASIBLE; return;             // (2.8)
        }
        F += prob_.targets[j].value * (1.0 - Sj);
    }
    sol.fitness = F;
}

// ---------------------------------------------------------------------------
//  SUA CHUA: dua ca the ve mien kha thi theo dung thu tu 4 buoc.
//  Thu tu nay khong doi cho duoc: R2 phai truoc R3 (de R3 chi bo sung tu UAV
//  thuc su ranh), va R1 phai sau cung (huy ca cum khong pha vo R2/R3/R4).
// ---------------------------------------------------------------------------
void UAVGAOptimizer::repair(AssignmentSolution& sol)
{
    int n = sol.nUavTypes, m = sol.nTargets;
    if (n <= 0 || m <= 0) return;
    if ((int)sol.x.size() != n * m) sol.x.assign(n * m, 0);

    // ---- BUOC 1 : (2.10) tam bay ----
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            if (prob_.uavs[i].aij[j] == 0) sol.at(i, j) = 0;

    // ---- BUOC 2 : (2.7) moi UAV toi da mot lan xuat kich ----
    for (int i = 0; i < n; ++i)
    {
        int cnt = 0, bestJ = -1;
        double bestE = -1.0;
        for (int j = 0; j < m; ++j)
        {
            if (sol.at(i, j) != 1) continue;
            ++cnt;
            double cij = (prob_.uavs[i].cij[j] > 1.0) ? prob_.uavs[i].cij[j] : 1.0;
            double e = prob_.targets[j].value * prob_.uavs[i].pij[j] / cij;
            if (e > bestE) { bestE = e; bestJ = j; }
        }
        if (cnt <= 1) continue;
        for (int j = 0; j < m; ++j)
            if (j != bestJ) sol.at(i, j) = 0;
    }

    // ---- BUOC 3 : (2.8)(2.9) du hoa luc - BO SUNG TRUOC, HUY CUM SAU ----
    std::vector<int> order(m);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return prob_.targets[a].priority < prob_.targets[b].priority;
        });

    for (int idx = 0; idx < m; ++idx)
    {
        int j = order[idx];
        double need = prob_.targets[j].explosive_required;   // w_j
        if (need <= 0.0) continue;

        double have = 0.0;
        for (int i = 0; i < n; ++i)
            if (sol.at(i, j) == 1) have += prob_.uavs[i].explosive;

        if (have <= 0.0) continue;   // muc tieu khong duoc chon: y_j = 0, van hop le
        if (have >= need) continue;  // da du hoa luc

        // Ung vien bo sung: UAV con RANH, kha dung voi j, co mang thuoc no
        struct Cand { int i; double w; double key; };
        std::vector<Cand> cands;
        for (int i = 0; i < n; ++i)
        {
            if (prob_.uavs[i].aij[j] == 0)      continue;
            if (prob_.uavs[i].explosive <= 0.0) continue;
            if (isBusy(sol, i, m))              continue;
            double w = prob_.uavs[i].explosive;
            cands.push_back({ i, w, prob_.uavs[i].cij[j] / w });  // re nhat tren 1 kg no
        }
        std::uniform_real_distribution<double> jitter(0.0, 0.15);
        for (auto& c : cands) c.key *= (1.0 + jitter(rng()));
        std::sort(cands.begin(), cands.end(),
            [](const Cand& a, const Cand& b) { return a.key < b.key; });

        for (auto& c : cands)
        {
            if (have >= need) break;
            sol.at(c.i, j) = 1;
            have += c.w;
        }

        // Khong the bo sung du -> huy toan bo cum, dat y_j = 0
        if (have < need)
            for (int i = 0; i < n; ++i) sol.at(i, j) = 0;
    }

    // ---- BUOC 4 : (2.6) ngan sach chien dich ----
    const double C = prob_.campaignBudget;
    if (C <= 0.0) return;
    while (true)
    {
        double cost = 0.0;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < m; ++j)
                if (sol.at(i, j) == 1) cost += prob_.uavs[i].cij[j];
        if (cost <= C + 1e-9) break;

        // Huy cum co hieu qua thap nhat:  v_j (1 - S_j) / tong chi phi cum
        int worstJ = -1;
        double worstE = std::numeric_limits<double>::max();
        for (int j = 0; j < m; ++j)
        {
            double Sj = 1.0, cj = 0.0;
            bool any = false;
            for (int i = 0; i < n; ++i)
            {
                if (sol.at(i, j) != 1) continue;
                Sj *= (1.0 - prob_.uavs[i].pij[j]);
                cj += prob_.uavs[i].cij[j];
                any = true;
            }
            if (!any || cj <= 0.0) continue;
            double e = prob_.targets[j].value * (1.0 - Sj) / cj;
            if (e < worstE) { worstE = e; worstJ = j; }
        }
        if (worstJ < 0) break;
        for (int i = 0; i < n; ++i) sol.at(i, worstJ) = 0;
    }
}

AssignmentSolution UAVGAOptimizer::run()
{
    rng();   // ep khoi tao bo sinh ngau nhien de lay dung seed
    std::cout << "[GA] Seed = " << currentSeed()
              << " | quan the = " << popSize_
              << " | the he = " << maxGen_ << "\n";
    initPopulation();

    // In gia tri tot nhat sau moi `step` the he de quan sat qua trinh hoi tu.
    const int step = (maxGen_ >= 200) ? 50 : (maxGen_ / 10 > 0 ? maxGen_ / 10 : 1);

    AssignmentSolution best = population_[0];
    for (auto& sol : population_)
        if (sol.fitness > best.fitness) best = sol;

    for (int gen = 0; gen < maxGen_; ++gen)
    {
        std::vector<AssignmentSolution> newPop;
        newPop.reserve(popSize_ + 1);
        newPop.push_back(best);                 // chien luoc tinh hoa
        for (int k = 0; k < popSize_; ++k)
        {
            AssignmentSolution child = crossover(selectParent(), selectParent());
            mutate(child);
            repair(child);
            evaluate(child);
            newPop.push_back(child);
            if (child.fitness > best.fitness) best = child;
        }
        population_ = std::move(newPop);

        if (g_gaVerbose && ((gen + 1) % step == 0 || gen + 1 == maxGen_))
            std::cout << "[GA] The he " << std::setw(4) << (gen + 1)
                      << " : F = " << std::fixed << std::setprecision(2)
                      << best.fitness << "\n";
    }
    return best;
}

// ===========================================================================
//  XAY DUNG BAI TOAN TU DU LIEU + CHAY TOI UU
// ===========================================================================
OptimizationProblem OptimizationBuilder::build(const UnitUAVList& unitList,
    const Graph& graph, const std::string& dataDir)
{
    OptimizationProblem prob;

    // ---- PHAN 0 : tham so chien dich ----
    loadConfig(dataDir + "\\Config.csv", prob);

    // ---- PHAN 1 : danh sach muc tieu ----
    for (const auto& t : graph.GetTargets())
    {
        TargetOpt to;
        to.id = t.target_id;
        to.code = t.code;
        to.name = t.name;
        to.value = t.value;
        to.x = t.x;
        to.y = t.y;
        to.vertexId = t.id_vertex;
        to.explosive_required = t.explosive;   // w_j
        to.priority = t.priority;
        prob.targets.push_back(to);

        std::cout << "[BUILD] Target " << to.id << " \"" << to.name << "\""
                  << " | vertex=" << to.vertexId
                  << " | w_j=" << to.explosive_required
                  << " | v_j=" << to.value
                  << " | priority=" << to.priority << "\n";
    }
    int m = (int)prob.targets.size();

    // ---- PHAN 2 : KHAI TRIEN TUNG CA THE UAV ----
    // Mot dong CSV co quantity = q sinh ra q ban ghi doc lap trong prob.uavs.
    // Nho vay so mu trong (1 - p_ij)^x_ij luon dung khi nhieu phuong tien
    // cung chung loai cung danh mot muc tieu.
    for (const auto& unit : unitList.getUnits())
    {
        for (const auto& u : unit.getUAVs())
        {
            if (u.getExplosive() <= 0.0) {
                std::cout << "[BUILD] Bo qua UAV " << u.getCode()
                          << " (explosive=0, khong tham gia GA)\n";
                continue;
            }

            int q = (u.getQuantity() >= 1) ? u.getQuantity() : 1;

            // He so hanh trinh tau: lay tu CSV neu co, neu khong suy tu tien to ma.
            // Quy uoc du lieu: B* = cam tu (bay mot chieu), C* = chien dau (khu hoi).
            int tau = u.getTau();
            if (tau != 1 && tau != 2) {
                const std::string& c = u.getCode();
                tau = (!c.empty() && (c[0] == 'B' || c[0] == 'b')) ? 1 : 2;
            }

            int   startV = unit.getVertexId();
            double rangeM = (double)u.getRange();   // R_i, don vi met

            for (int k = 0; k < q; ++k)
            {
                UAVOpt opt;
                opt.id = u.getId() * 100 + (k + 1);
                opt.baseCode = u.getCode();
                opt.code = u.getCode() + "-" + std::to_string(k + 1);
                opt.type = u.getType();
                opt.instance = k + 1;
                opt.tau = tau;
                opt.range = rangeM;
                opt.speed = u.getSpeed();
                opt.explosive = u.getExplosive();          // w_i
                opt.ValuePerAttack = u.getCost();          // c_i^0
                opt.unitIndex = unitList.getUnitIndex(unit.getUnitId());
                opt.unitName = unit.getUnitName();
                opt.aij.assign(m, 1);
                opt.pij.assign(m, 0.0);
                opt.cij.assign(m, std::numeric_limits<double>::max());

                for (int j = 0; j < m; ++j)
                {
                    // L_ij : duong di ngan nhat tren do thi (met)
                    double L = graph.shortestPathDistance(startV, prob.targets[j].vertexId);

                    // (2.2)  a_ij = 1  <=>  tau_i * L_ij <= R_i
                    opt.aij[j] = (tau * L <= rangeM) ? 1 : 0;

                    // (2.1)  c_ij = c_i^0 + k * L_ij / 1000
                    opt.cij[j] = opt.ValuePerAttack + prob.costCoefK * (L / 1000.0);
                }
                prob.uavs.push_back(opt);
            }

            std::cout << "[BUILD] UAV " << u.getCode() << " x" << q
                      << " | don_vi=" << unit.getUnitName()
                      << " | w_i=" << u.getExplosive()
                      << " | c_i0=" << u.getCost()
                      << " | tau=" << tau
                      << " | R=" << rangeM / 1000.0 << " km\n";
        }
    }

    int n = (int)prob.uavs.size();
    std::cout << "[BUILD] Tong: n = " << n << " UAV ca the, m = " << m << " muc tieu\n";

    // ---- PHAN 3 : gan xac suat p_ij (tra cuu theo MA CHUNG LOAI) ----
    auto pijMap = loadPij(dataDir + "\\Probability.csv");
    for (auto& uav : prob.uavs)
        for (int j = 0; j < m; ++j)
        {
            std::string key = uav.baseCode + "|" + std::to_string(prob.targets[j].id);
            uav.pij[j] = pijMap.count(key) ? pijMap[key] : 0.0;
        }

    // Canh bao khi rang buoc tam bay khong co hieu luc
    {
        bool allAvail = true;
        for (const auto& u : prob.uavs)
            for (int j = 0; j < m; ++j)
                if (u.aij[j] == 0) { allAvail = false; break; }
        if (allAvail)
            std::cout << "[BUILD] LUU Y: a_ij = 1 voi moi cap -> rang buoc (2.10) "
                         "khong co hieu luc tren bo du lieu hien tai.\n";
    }

    // ---- PHAN 4 : chay thuat toan di truyen ----
    // ga_runs = 1 : chay mot lan, dung de lap ke hoach tien cong.
    // ga_runs > 1 : chay lap doc lap va in thong ke - so lieu danh gia chat
    //               luong thuat toan dua thang vao bao cao.
    const int nRuns = (prob.gaRuns > 1) ? prob.gaRuns : 1;
    g_gaVerbose = (nRuns == 1);   // chay 30 lan thi khong in tien trinh cho do roi
    AssignmentSolution best;
    {
        std::vector<double> fs;
        fs.reserve(nRuns);
        for (int r = 0; r < nRuns; ++r)
        {
            // Hat giong co dinh: lan chay thu r dung ga_seed + r, nho vay ca
            // mot lan chay lan ca bo nRuns lan deu tai lap duoc y nguyen.
            if (prob.gaSeed != 0u) seedRng(prob.gaSeed + (uint32_t)r);

            UAVGAOptimizer ga(prob, /*popSize*/200, /*maxGen*/500,
                                    /*pc*/0.85,     /*pm*/0.10);
            AssignmentSolution cur = ga.run();
            fs.push_back(cur.fitness);
            if (r == 0 || cur.fitness > best.fitness) best = cur;

            if (nRuns > 1)
                std::cout << "[GA] Lan " << (r + 1) << "/" << nRuns
                          << " : F = " << cur.fitness << "\n";
        }

        if (nRuns > 1)
        {
            double sum = 0.0, mn = fs[0], mx = fs[0];
            for (double v : fs) { sum += v; if (v < mn) mn = v; if (v > mx) mx = v; }
            const double mean = sum / nRuns;
            double var = 0.0;
            for (double v : fs) var += (v - mean) * (v - mean);
            const double sd = std::sqrt(var / (nRuns - 1));
            int hit = 0;
            for (double v : fs) if (v > mx - 1e-6) ++hit;

            std::cout << std::fixed
                << "[GA] ----- Thong ke " << nRuns << " lan chay doc lap -----\n"
                << "[GA] Tot nhat      = " << std::setprecision(4) << mx   << "\n"
                << "[GA] Trung binh    = " << std::setprecision(4) << mean << "\n"
                << "[GA] Xau nhat      = " << std::setprecision(4) << mn   << "\n"
                << "[GA] Do lech chuan = " << std::setprecision(4) << sd   << "\n"
                << "[GA] Sai lech xau nhat so voi tot nhat = "
                << std::setprecision(3) << (mx > 0.0 ? (mx - mn) / mx * 100.0 : 0.0) << " %\n"
                << "[GA] So lan dat muc tot nhat = " << hit << "/" << nRuns << "\n";
        }
        else
        {
            std::cout << "[GA] Hoan thanh. F = " << best.fitness << "\n";
        }
    }

    // ---- PHAN 4b : THUAT TOAN DOI CHUNG (muc 3.4) ----
    // Chi chay khi Config.csv dat `run_exact,1`. Mac dinh TAT: quy hoach dong
    // la cong cu kiem chung, khong nam tren duong chay tac chien (muc 3.4.7).
    if (prob.runExact)
    {
        ExactSolution ex = solveExact(prob);
        if (!ex.ok)
            std::cout << "[DP] Khong giai duoc: " << ex.note << "\n";
        else
        {
            const double gap = (ex.value > 1e-9)
                             ? 100.0 * (ex.value - best.fitness) / ex.value : 0.0;
            std::cout << std::fixed << std::setprecision(4)
                      << "[DP] F* = " << ex.value
                      << " | chi phi = " << std::setprecision(2) << ex.cost
                      << " | " << ex.nStates << " trang thai, "
                      << ex.nPackages << " goi"
                      << " | " << ex.ms << " ms\n";
            std::cout << std::setprecision(4)
                      << "[SS] Sai lech cua thuat toan di truyen = " << gap << " %\n";
            prob.hasExact   = true;
            prob.exactValue = ex.value;
            prob.exactGap   = gap;
            prob.exactMs    = ex.ms;
        }
    }

    // ---- PHAN 5 : KIEM DINH DOC LAP cac rang buoc (2.6)-(2.10) ----
    {
        bool ok = true;
        double cost = 0.0;
        for (int i = 0; i < n; ++i)
        {
            int cnt = 0;
            for (int j = 0; j < m; ++j)
            {
                if (best.x[i * m + j] != 1) continue;
                ++cnt;
                cost += prob.uavs[i].cij[j];
                if (prob.uavs[i].aij[j] == 0) {
                    std::cout << "[CHECK] VI PHAM (2.10): " << prob.uavs[i].code
                              << " -> " << prob.targets[j].name << "\n"; ok = false;
                }
            }
            if (cnt > 1) {
                std::cout << "[CHECK] VI PHAM (2.7): " << prob.uavs[i].code
                          << " nhan " << cnt << " nhiem vu\n"; ok = false;
            }
        }
        if (prob.campaignBudget > 0.0 && cost > prob.campaignBudget + 1e-6) {
            std::cout << "[CHECK] VI PHAM (2.6): chi phi " << cost
                      << " > C = " << prob.campaignBudget << "\n"; ok = false;
        }
        for (int j = 0; j < m; ++j)
        {
            double have = 0.0; bool any = false;
            for (int i = 0; i < n; ++i)
                if (best.x[i * m + j] == 1) { have += prob.uavs[i].explosive; any = true; }
            if (any && have < prob.targets[j].explosive_required - 1e-9) {
                std::cout << "[CHECK] VI PHAM (2.8): " << prob.targets[j].name
                          << " co " << have << " kg < w_j = "
                          << prob.targets[j].explosive_required << "\n"; ok = false;
            }
        }
        std::cout << "[CHECK] " << (ok ? "Phuong an THOA MAN toan bo rang buoc."
                                       : "!!! Phuong an VI PHAM rang buoc.")
                  << " Tong chi phi = " << cost;
        if (prob.campaignBudget > 0.0) std::cout << " / " << prob.campaignBudget;
        std::cout << " USD\n";
    }

    // ---- PHAN 6 : duong bay Dijkstra ----
    best.paths.assign(n, std::vector<std::vector<int>>(m));
    for (int i = 0; i < n; ++i)
    {
        const UAVOpt& uav = prob.uavs[i];
        int startV = unitList.getUnit(uav.unitIndex).getVertexId();
        for (int j = 0; j < m; ++j)
        {
            if (best.x[i * m + j] != 1) continue;
            int endV = prob.targets[j].vertexId;
            std::vector<int> path = graph.shortestPath(startV, endV);
            if (path.empty()) {
                std::cout << "[DIJKSTRA] CANH BAO: Khong tim duoc duong "
                          << startV << "->" << endV << " (UAV " << uav.code << ")\n";
                path = { startV, endV };
            }
            best.paths[i][j] = path;
            std::cout << "[DIJKSTRA] " << uav.code << " (" << uav.unitName << ")"
                      << " TAN CONG " << prob.targets[j].name
                      << " | " << startV << "->" << endV
                      << " | " << path.size() << " dinh\n";
        }
    }

    // ---- PHAN 7 : ket qua ----
    best.unitIndex.resize(n);
    for (int i = 0; i < n; ++i) best.unitIndex[i] = prob.uavs[i].unitIndex;

    std::cout << "[KET QUA] Muc tieu duoc tien cong (y_j = 1): ";
    for (int j = 0; j < m; ++j)
        if (best.y(j)) std::cout << prob.targets[j].name << "; ";
    std::cout << "\n[KET QUA] Muc tieu bi loai (y_j = 0): ";
    for (int j = 0; j < m; ++j)
        if (!best.y(j)) std::cout << prob.targets[j].name << "; ";
    std::cout << "\n";

    // ---- PHAN 8 : XUAT KE HOACH TIEN CONG CHO MO PHONG BA CHIEU ----
    // Chuong trinh hai chieu nay la NOI DUY NHAT giai bai toan toi uu.
    // Du an Unreal Engine khong chay lai thuat toan; no chi doc hai tep
    // sinh ra o day roi dung hinh. Nho vay hai chuong trinh khong the
    // cho ra hai ket qua khac nhau.
    // Toan bo noi dung hai tep deu la ASCII, tranh loi ma tieng Viet khi
    // Unreal Engine doc tep khong co dau BOM.
    {
        const std::string planPath = dataDir + "\\MissionPlan.csv";
        const std::string sumPath  = dataDir + "\\MissionSummary.csv";

        std::ofstream fp(planPath, std::ios::binary);
        if (!fp)
        {
            std::cout << "[XUAT] LOI: khong ghi duoc " << planPath << "\n";
        }
        else
        {
            fp << "sortie_id,uav_code,base_code,tau,unit_id,start_vertex,"
                  "target_id,target_vertex,p_ij,c_ij,w_i,speed,path\r\n";

            int    sortie    = 0;
            double totalCost = 0.0;

            for (int i = 0; i < n; ++i)
            {
                const UAVOpt&     uav    = prob.uavs[i];
                const std::string unitId = unitList.getUnit(uav.unitIndex).getUnitId();
                const int         startV = unitList.getUnit(uav.unitIndex).getVertexId();

                for (int j = 0; j < m; ++j)
                {
                    if (best.x[i * m + j] != 1) continue;
                    ++sortie;
                    totalCost += uav.cij[j];

                    // Chuoi dinh cua duong bay Dijkstra, ngan cach bang dau |
                    std::string pathStr;
                    const std::vector<int>& pv = best.paths[i][j];
                    for (size_t t = 0; t < pv.size(); ++t)
                    {
                        if (t) pathStr += "|";
                        pathStr += std::to_string(pv[t]);
                    }

                    fp << sortie                     << ','
                       << uav.code                   << ','
                       << uav.baseCode               << ','
                       << uav.tau                    << ','
                       << unitId                     << ','
                       << startV                     << ','
                       << prob.targets[j].id         << ','
                       << prob.targets[j].vertexId   << ','
                       << std::fixed
                       << std::setprecision(4) << uav.pij[j]     << ','
                       << std::setprecision(2) << uav.cij[j]     << ','
                       << std::setprecision(2) << uav.explosive  << ','
                       << std::setprecision(1) << uav.speed      << ','
                       << pathStr << "\r\n";
                }
            }
            fp.close();
            std::cout << "[XUAT] MissionPlan.csv : " << sortie << " luot xuat kich\n";

            // --- Tom tat cap chien dich ---
            int    nAttacked  = 0;
            double totalValue = 0.0;
            for (int j = 0; j < m; ++j)
            {
                totalValue += prob.targets[j].value;
                if (best.y(j)) ++nAttacked;
            }

            std::ofstream fs(sumPath, std::ios::binary);
            if (fs)
            {
                fs << "key,value\r\n" << std::fixed;
                fs << "F_objective,"        << std::setprecision(4) << best.fitness        << "\r\n";
                fs << "total_cost,"         << std::setprecision(2) << totalCost           << "\r\n";
                fs << "campaign_budget,"    << std::setprecision(2) << prob.campaignBudget << "\r\n";
                fs << "cost_coef_k,"        << std::setprecision(2) << prob.costCoefK      << "\r\n";
                fs << "total_target_value," << std::setprecision(2) << totalValue          << "\r\n";
                fs << "n_uav_total,"        << n         << "\r\n";
                fs << "n_uav_used,"         << sortie    << "\r\n";
                fs << "n_target_total,"     << m         << "\r\n";
                fs << "n_target_attacked,"  << nAttacked << "\r\n";
                fs << "ga_seed,"            << currentSeed() << "\r\n";
                if (prob.hasExact)
                {
                    fs << "F_optimal,"      << std::setprecision(4) << prob.exactValue << "\r\n";
                    fs << "gap_percent,"    << std::setprecision(4) << prob.exactGap   << "\r\n";
                }
                fs.close();
                std::cout << "[XUAT] MissionSummary.csv : F = " << best.fitness
                          << " | chi phi = " << totalCost << " USD\n";
            }
            else
            {
                std::cout << "[XUAT] LOI: khong ghi duoc " << sumPath << "\n";
            }
        }
    }

    // ---- PHAN 9 : DONG BO SANG DU AN BA CHIEU ----
    // Neu Config.csv co khai bao `ue_data_dir`, chep thang ke hoach va bo du
    // lieu sang thu muc Content\Data cua du an Unreal Engine. Nho vay chi can
    // chay chuong trinh nay la du an ba chieu da co du lieu moi nhat, khong
    // phai chep tay hay chay them tep .bat nao.
    if (!prob.ueDataDir.empty())
    {
        std::string dst = prob.ueDataDir;
        if (dst.back() != '\\' && dst.back() != '/') dst += "\\";

        struct Pair { const char* from; const char* to; };
        const Pair files[] = {
            { "MissionPlan.csv",    "MissionPlan.csv"    },
            { "MissionSummary.csv", "MissionSummary.csv" },
            { "Vertex.csv",         "Vertex1.csv"        },
            { "Edge.csv",           "Edge1.csv"          },
            { "UnitUAV.csv",        "UnitUAV1.csv"       },
            { "Data_target.csv",    "Data_target1.csv"   },
            { "Data_uav.csv",       "Data_uav1.csv"      },
            { "Probability.csv",    "Probability1.csv"   },
        };

        int ok = 0, fail = 0;
        for (const auto& f : files)
        {
            if (copyFileBinary(dataDir + "\\" + f.from, dst + f.to)) ++ok;
            else {
                ++fail;
                std::cout << "[DONGBO] LOI: khong ghi duoc " << dst << f.to << "\n";
            }
        }
        std::cout << "[DONGBO] Da chep " << ok << "/" << (ok + fail)
                  << " tep sang du an ba chieu.\n";
        if (fail > 0)
            std::cout << "[DONGBO] LUU Y: Unreal Editor dang mo se khoa tep. "
                         "Dong Editor roi chay lai chuong trinh nay.\n";
    }
    else
    {
        std::cout << "[DONGBO] Config.csv chua co khoa `ue_data_dir` "
                     "-> bo qua buoc dong bo.\n";
    }

    prob.bestSolution = best;
    return prob;
}
