#pragma once
#pragma once

#include <vector>
#include <string>

// Một loại UAV dùng cho bài toán tối ưu (không phải UAV đơn lẻ)
struct UAVTypeOpt
{
    int id;                 // chỉ số i
    std::string code;       // tên/loại UAV (VD: VU-C2-01)
    double costPerAttack;   // c_ij (ở đây tạm coi c_ij = costPerAttack, nếu cần chi tiết hơn thì dùng ma trận)
    int maxCount;           // M_i: số lượng UAV loại i
    double maxBudget;       // C_i: tổng chi phí tối đa cho loại i

    // Các vector theo target j
    std::vector<double> pij; // xác suất tiêu diệt target j
    std::vector<int> aij;    // 0/1: có thể tấn công target j không
    int unitIndex;      // đơn vị mà UAV này thuộc về
    std::string unitName; // tên đơn vị, ví dụ "SQ1"
};

// Thông tin mục tiêu cho bài toán tối ưu
struct TargetOpt
{
    int id;              // chỉ số j
    std::string code;    // mã mục tiêu (T-01, T-02,...)
    double value;        // v_j: giá trị mục tiêu
    double x;   
    double y;
};

// Dữ liệu đầy đủ cho bài toán tối ưu
struct OptimizationProblem
{
    std::vector<UAVTypeOpt> uavs;   // I
    std::vector<TargetOpt> targets; // J
};

// Một nghiệm (cá thể) của GA: ma trận x_ij
struct AssignmentSolution {
    int nUavTypes;
    int nTargets;
    std::vector<int> x;
    double fitness;
    std::vector<UAVTypeOpt> uavs;

    int at(int i, int j) const {
        return x[i * nTargets + j];      // ⭐ dùng nTargets
    }
    int& at(int i, int j) {
        return x[i * nTargets + j];      // ⭐ dùng nTargets
    }
};



// Lớp GA
class UAVGAOptimizer
{
public:
    UAVGAOptimizer(const OptimizationProblem& problem,
        int populationSize,
        int maxGenerations,
        double crossoverRate,
        double mutationRate);

    AssignmentSolution run(); // chạy GA, trả về nghiệm tốt nhất

private:
    const OptimizationProblem& prob_;
    int popSize_;
    int maxGen_;
    double pc_;
    double pm_;

    std::vector<AssignmentSolution> population_;

    void initPopulation();
    void evaluate(AssignmentSolution& sol);
    AssignmentSolution selectParent();
    AssignmentSolution crossover(const AssignmentSolution& p1, const AssignmentSolution& p2);
    void mutate(AssignmentSolution& child);
    void repair(AssignmentSolution& sol); // sửa ràng buộc C_i, M_i
};
