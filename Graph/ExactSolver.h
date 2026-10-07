#pragma once
#include "OptimizationTypes.h"
#include <vector>
#include <string>

// ---------------------------------------------------------------------------
//  THUAT TOAN DOI CHUNG - QUY HOACH DONG THEO GOI TIEN CONG
//  Cai dat cua thuat toan trinh bay o muc 3.4 cua bao cao.
//  Cho NGHIEM TOI UU TUYET DOI, dung lam chuan do sai lech cua thuat toan
//  di truyen. KHONG nam tren duong chay tac chien: chi chay khi Config.csv
//  dat `run_exact,1`.
// ---------------------------------------------------------------------------
struct ExactSolution
{
    bool        ok        = false;  // false : vuot nguong quy mo, khong giai
    double      value     = 0.0;    // F* : gia tri toi uu tuyet doi
    double      cost      = 0.0;    // tong chi phi cua phuong an toi uu
    long long   ms        = 0;      // thoi gian tinh (mili giay)
    long long   nStates   = 0;      // so trang thai quy hoach dong
    long long   nPackages = 0;      // tong so goi tien cong sau khi cat troi

    std::vector<std::string>      typeName;  // ten cac chung loai hoan vi duoc
    std::vector<int>              typeQty;   // so luong moi chung loai
    std::vector<std::vector<int>> plan;      // plan[j][t] : so phuong tien
                                             // chung loai t giao cho muc tieu j
    std::string note;                        // ly do khi ok = false
};

// maxStates : nguong an toan. Vuot qua thi tra ve ok = false kem ghi chu,
// dung de minh hoa gioi han quy mo neu o muc 3.4.7.
ExactSolution solveExact(const OptimizationProblem& prob,
                         long long maxStates = 5000000LL);
