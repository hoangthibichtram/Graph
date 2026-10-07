#pragma once
#include <vector>
#include <string>

// ---------------------------------------------------------------------------
// Mot muc tieu can tien cong.
// Ky hieu trong mo hinh: v_j = value, w_j = explosive_required
// ---------------------------------------------------------------------------
struct TargetOpt
{
    int id;
    std::string code;
    std::string name;
    double value;               // v_j : gia tri quan su (USD)
    double x, y;
    int vertexId;
    std::string type;
    double explosive_required;  // w_j : luong no toi thieu de vo hieu hoa (kg)
    int priority;
};

// ---------------------------------------------------------------------------
// Mot UAV CA THE - khong phai mot chung loai.
// Moi dong trong Data_uav.csv co quantity = q se sinh ra q ban ghi kieu nay.
// Day la dieu kien can de cong thuc  S_j = PROD (1 - p_ij)^x_ij  voi
// x_ij nhi phan giu duoc tinh chinh xac khi nhieu phuong tien cung chung loai
// cung danh mot muc tieu.
// ---------------------------------------------------------------------------
struct UAVOpt
{
    int id;
    std::string code;        // ma hien thi, vi du "B11-2"
    std::string baseCode;    // ma chung loai "B11" - dung tra cuu Probability.csv
    std::string type;
    int instance = 1;        // so thu tu ca the trong chung loai (1..q)
    int tau = 1;             // he so hanh trinh: 1 = cam tu, 2 = chien dau (khu hoi)
    double range = 0.0;      // R_i (m)
    double speed = 0.0;
    double explosive = 0.0;  // w_i (kg)
    int unitIndex = -1;
    double ValuePerAttack = 0.0;  // c_i^0 : chi phi co dinh mot lan xuat kich (USD)
    std::string unitName;

    std::vector<int>    aij;  // a_ij : kha dung theo tam bay
    std::vector<double> pij;  // p_ij : xac suat tieu diet
    std::vector<double> cij;  // c_ij = c_i^0 + k * L_ij / 1000
};

// Giu ten cu de cac file khac khong phai sua dong loat
using UAVTypeOpt = UAVOpt;

struct AssignmentSolution
{
    int nUavTypes{};   // = n : SO UAV CA THE (giu ten cu de tuong thich)
    int nTargets{};    // = m
    std::vector<int> x;
    double fitness{};
    std::vector<int> unitIndex;
    std::vector<std::vector<std::vector<int>>> paths;

    int  at(int i, int j) const { return x[i * nTargets + j]; }
    int& at(int i, int j)       { return x[i * nTargets + j]; }

    // y_j : muc tieu j co duoc dua vao ke hoach tien cong hay khong
    int y(int j) const
    {
        for (int i = 0; i < nUavTypes; ++i)
            if (x[i * nTargets + j] == 1) return 1;
        return 0;
    }
};

class OptimizationProblem
{
public:
    std::vector<TargetOpt> targets;
    std::vector<UAVOpt>    uavs;

    double costCoefK      = 0.5;  // k : don gia hanh trinh (USD/km)
    double campaignBudget = 0.0;  // C : ngan sach chien dich (USD). <= 0 : khong gioi han

    // Thu muc du lieu cua du an ba chieu (Content\Data). Khai bao trong
    // Config.csv bang khoa `ue_data_dir`. De trong thi bo qua buoc dong bo.
    std::string ueDataDir;

    // Hat giong sinh so ngau nhien cho thuat toan di truyen, khoa `ga_seed`.
    // = 0 : lay ngau nhien tu dong ho, moi lan chay mot ket qua khac.
    // > 0 : co dinh, moi lan chay cho dung mot ket qua - dung khi can tai lap.
    unsigned gaSeed = 0u;

    // So lan chay lap thuat toan di truyen, khoa `ga_runs`.
    // = 1 : chay mot lan, dung de lap ke hoach.
    // > 1 : chay lap va in thong ke, dung de danh gia chat luong thuat toan.
    int gaRuns = 1;
    // Bat thuat toan doi chung (quy hoach dong, muc 3.4), khoa `run_exact`.
    // = false : chi chay thuat toan di truyen - che do lap ke hoach tac chien.
    // = true  : chay them quy hoach dong, in F* va sai lech - che do kiem chung.
    bool runExact = false;
    // Ket qua cua thuat toan doi chung, chi co nghia khi hasExact = true.
    bool   hasExact   = false;
    double exactValue = 0.0;   // F* : gia tri toi uu tuyet doi
    double exactGap   = 0.0;   // sai lech cua thuat toan di truyen (%)
    long long exactMs = 0;     // thoi gian chay doi chung (ms)

    AssignmentSolution bestSolution;
};
