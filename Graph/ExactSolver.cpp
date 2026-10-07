#include "ExactSolver.h"
#include <map>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <limits>

namespace {

// Hai phuong tien HOAN VI DUOC cho nhau khi va chi khi chung giong nhau o
// toan bo dai luong di vao mo hinh: luong no mang theo, va ba vec-to a_ij,
// p_ij, c_ij. Ta dung chinh bo dai luong do lam khoa gom nhom, nen dieu kien
// hoan vi duoc luon dung THEO CAU TAO, khong phai gia thiet.
std::string groupKey(const UAVOpt& u, int m)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6) << u.explosive << '#';
    for (int j = 0; j < m; ++j) ss << (j < (int)u.aij.size() ? u.aij[j] : 0) << ',';
    ss << '#';
    for (int j = 0; j < m; ++j) ss << (j < (int)u.pij.size() ? u.pij[j] : 0.0) << ',';
    ss << '#';
    for (int j = 0; j < m; ++j) ss << (j < (int)u.cij.size() ? u.cij[j] : 0.0) << ',';
    return ss.str();
}

struct Pkg
{
    std::vector<int> cnt;   // so phuong tien lay tu moi chung loai
    double cost;
    double val;
};

struct Entry
{
    double cost;
    double val;
    std::vector<int> pick;  // chi so goi da chon cho tung muc tieu (-1 = bo qua)
};

// Cat troi Pareto: sap theo chi phi tang dan, chi giu diem co gia tri cao hon
// moi diem re hon no. Bo di deu la diem bi troi, khong the nam trong nghiem
// toi uu (chung minh o muc 3.4.6).
// Mot goi bi TROI khi ton tai goi khac dong thoi: re hon (hoac bang), gia tri
// cao hon (hoac bang), VA dung khong nhieu hon o TUNG chung loai. Thieu dieu
// kien thu ba thi se loai nham nhung goi dat hon nhung tiet kiem phuong tien
// khan hiem - va nhung goi do van co the nam trong nghiem toi uu.
void prunePkg(std::vector<Pkg>& v)
{
    std::sort(v.begin(), v.end(), [](const Pkg& a, const Pkg& b) {
        if (a.cost != b.cost) return a.cost < b.cost;
        return a.val > b.val;
    });
    std::vector<Pkg> out;
    for (size_t i = 0; i < v.size(); ++i)
    {
        bool dominated = false;
        for (size_t k = 0; k < out.size() && !dominated; ++k)
        {
            if (out[k].cost > v[i].cost + 1e-9) continue;
            if (out[k].val  < v[i].val  - 1e-9) continue;
            bool le = true;
            for (size_t t = 0; t < v[i].cnt.size(); ++t)
                if (out[k].cnt[t] > v[i].cnt[t]) { le = false; break; }
            if (le) dominated = true;
        }
        if (!dominated) out.push_back(v[i]);
    }
    v.swap(out);
}

void pruneEntry(std::vector<Entry>& v)
{
    std::sort(v.begin(), v.end(), [](const Entry& a, const Entry& b) {
        if (a.cost != b.cost) return a.cost < b.cost;
        return a.val > b.val;
    });
    std::vector<Entry> out;
    double best = -1e300;
    for (size_t i = 0; i < v.size(); ++i)
        if (v[i].val > best + 1e-12) { out.push_back(v[i]); best = v[i].val; }
    v.swap(out);
}

} // namespace

ExactSolution solveExact(const OptimizationProblem& prob, long long maxStates)
{
    using clk = std::chrono::steady_clock;
    const clk::time_point t0 = clk::now();

    ExactSolution R;
    const int m = (int)prob.targets.size();
    const int n = (int)prob.uavs.size();
    if (m == 0 || n == 0) { R.note = "Bai toan rong"; return R; }

    // ---- BUOC 0 : gom cac phuong tien hoan vi duoc thanh chung loai --------
    std::map<std::string, int> idx;
    std::vector<int>    qty;      // q_t
    std::vector<double> wt;       // w_t : luong no mang theo
    std::vector<std::vector<int> >    av;   // a_tj
    std::vector<std::vector<double> > pv;   // p_tj
    std::vector<std::vector<double> > cv;   // c_tj

    for (int i = 0; i < n; ++i)
    {
        const UAVOpt& u = prob.uavs[i];
        const std::string k = groupKey(u, m);
        std::map<std::string, int>::iterator it = idx.find(k);
        if (it == idx.end())
        {
            idx[k] = (int)qty.size();
            qty.push_back(1);
            wt.push_back(u.explosive);
            std::vector<int>    a(m, 0);
            std::vector<double> p(m, 0.0), c(m, 0.0);
            for (int j = 0; j < m; ++j)
            {
                if (j < (int)u.aij.size()) a[j] = u.aij[j];
                if (j < (int)u.pij.size()) p[j] = u.pij[j];
                if (j < (int)u.cij.size()) c[j] = u.cij[j];
            }
            av.push_back(a); pv.push_back(p); cv.push_back(c);
            R.typeName.push_back(u.baseCode);
        }
        else qty[it->second] += 1;
    }
    const int T = (int)qty.size();
    R.typeQty = qty;

    // ---- kiem tra nguong quy mo ------------------------------------------
    // So trang thai toi da = TICH (q_t + 1). Dai luong nay tang theo HAM MU
    // cua so chung loai - dung la gioi han neu o muc 3.4.7.
    double est = 1.0;
    for (int t = 0; t < T; ++t) est *= (double)(qty[t] + 1);
    if (est > (double)maxStates)
    {
        std::ostringstream ss;
        ss << "Vuot nguong quy mo: " << T << " chung loai cho "
           << (long long)est << " trang thai (nguong " << maxStates << ")";
        R.note = ss.str();
        R.ms = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                   clk::now() - t0).count();
        return R;
    }

    const double B = (prob.campaignBudget > 0.0)
                   ? prob.campaignBudget : std::numeric_limits<double>::max() / 4.0;

    // ---- BUOC 1 + 2 : sinh goi tien cong cho tung muc tieu, cat troi ------
    std::vector<std::vector<Pkg> > PK(m);
    for (int j = 0; j < m; ++j)
    {
        const double wj = prob.targets[j].explosive_required;
        const double vj = prob.targets[j].value;

        std::vector<int> hi(T, 0);
        for (int t = 0; t < T; ++t) hi[t] = av[t][j] ? qty[t] : 0;

        std::vector<int> cnt(T, 0);
        for (;;)
        {
            int s = 0;
            for (int t = 0; t < T; ++t) s += cnt[t];
            if (s > 0)
            {
                double ex = 0.0, cs = 0.0, surv = 1.0;
                for (int t = 0; t < T; ++t)
                    if (cnt[t])
                    {
                        ex   += cnt[t] * wt[t];
                        cs   += cnt[t] * cv[t][j];
                        surv *= std::pow(1.0 - pv[t][j], (double)cnt[t]);
                    }
                // Rang buoc hoa luc (2.8): duoi nguong thi cum khong sinh gia tri
                if (ex + 1e-9 >= wj && cs <= B + 1e-9)
                {
                    Pkg g; g.cnt = cnt; g.cost = cs; g.val = vj * (1.0 - surv);
                    PK[j].push_back(g);
                }
            }
            int t = 0;
            while (t < T && cnt[t] == hi[t]) { cnt[t] = 0; ++t; }
            if (t == T) break;
            ++cnt[t];
        }
        prunePkg(PK[j]);
        R.nPackages += (long long)PK[j].size();
    }

    // ---- BUOC 3 : quy hoach dong duyet lan luot cac muc tieu --------------
    std::map<std::vector<int>, std::vector<Entry> > cur;
    {
        Entry e; e.cost = 0.0; e.val = 0.0; e.pick.assign(m, -1);
        cur[std::vector<int>(T, 0)].push_back(e);
    }

    for (int j = 0; j < m; ++j)
    {
        std::map<std::vector<int>, std::vector<Entry> > nxt;
        for (std::map<std::vector<int>, std::vector<Entry> >::iterator it = cur.begin();
             it != cur.end(); ++it)
        {
            const std::vector<int>& used = it->first;

            // lua chon 1 : BO QUA muc tieu j  (y_j = 0, luon hop le)
            for (size_t e = 0; e < it->second.size(); ++e)
                nxt[used].push_back(it->second[e]);

            // lua chon 2 : chon mot goi tien cong cho muc tieu j
            for (size_t g = 0; g < PK[j].size(); ++g)
            {
                const Pkg& pg = PK[j][g];
                std::vector<int> nu(T);
                bool okq = true;
                for (int t = 0; t < T; ++t)
                {
                    nu[t] = used[t] + pg.cnt[t];
                    if (nu[t] > qty[t]) { okq = false; break; }   // rang buoc (2.7)
                }
                if (!okq) continue;
                for (size_t e = 0; e < it->second.size(); ++e)
                {
                    const Entry& be = it->second[e];
                    const double nc = be.cost + pg.cost;
                    if (nc > B + 1e-9) continue;                  // rang buoc (2.6)
                    Entry ne;
                    ne.cost = nc;
                    ne.val  = be.val + pg.val;
                    ne.pick = be.pick;
                    ne.pick[j] = (int)g;
                    nxt[nu].push_back(ne);
                }
            }
        }
        for (std::map<std::vector<int>, std::vector<Entry> >::iterator it = nxt.begin();
             it != nxt.end(); ++it)
            pruneEntry(it->second);
        cur.swap(nxt);
    }

    // ---- lay nghiem tot nhat tren toan bo bien Pareto cuoi cung -----------
    const Entry* best = 0;
    for (std::map<std::vector<int>, std::vector<Entry> >::iterator it = cur.begin();
         it != cur.end(); ++it)
    {
        R.nStates += 1;
        for (size_t e = 0; e < it->second.size(); ++e)
            if (!best || it->second[e].val > best->val) best = &it->second[e];
    }

    R.plan.assign(m, std::vector<int>(T, 0));
    if (best)
    {
        R.value = best->val;
        R.cost  = best->cost;
        for (int j = 0; j < m; ++j)
            if (best->pick[j] >= 0) R.plan[j] = PK[j][best->pick[j]].cnt;
    }
    R.ok = true;
    R.ms = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
               clk::now() - t0).count();
    return R;
}
