#pragma once
#include <string>
#include <functional>
#include "Graph.h"
#include "OptimizationProblem.h"
#include "OptimizationBuilder.h"

namespace UAVCore {

    // 1. Cấu trúc Cấu hình: Giúp truyền tham số mạnh mẽ và rõ ràng hơn
    struct EngineConfig {
        std::string unitFile;
        std::string vertexFile;
        std::string edgeFile;
        std::string targetFile;
        std::string unitsFolder;
        std::string uavPrefix = "UAV_";
        std::string uavExt = ".csv";
    };

    class UAVMissionEngine
    {
    public:
        UAVMissionEngine() = default;

        // Định nghĩa 1 kiểu hàm uỷ quyền (Callback) để xử lý việc in Log
        using LogCallback = std::function<void(const std::string&)>;

        // Đăng ký hệ thống Log (từ Win32 hoặc Unreal Engine)
        void SetLogger(LogCallback logger);

        // Khởi tạo bằng cấu trúc Config chuẩn
        bool InitEngine(const EngineConfig& config);

        // Khởi tạo nhanh từ một thư mục (Helper tự sinh đường dẫn)
        bool InitEngineFromDirectory(const std::string& dataDirectory);

        // Chạy bộ phân tích và tối ưu hóa
        bool RunOptimization();

        // API lấy kết quả và dữ liệu
        Graph& GetGraph() { return m_graph; }
        const AssignmentSolution& GetBestSolution() const { return m_bestSolution; }
        const OptimizationProblem& GetProblem() const { return m_problem; }

    private:
        // Hàm nội bộ gọi đến Logger đã đăng ký
        void PrintLog(const std::string& msg);

        Graph m_graph;
        OptimizationProblem m_problem;
        AssignmentSolution m_bestSolution;
        LogCallback m_logger = nullptr; // Chứa hàm log được bên ngoài cung cấp
    };

} // kết thúc namespace UAVCore
