#pragma once
#include <string>
#include <functional>
#include <unordered_map>
#include "Graph.h"
#include "OptimizationProblem.h"
#include "OptimizationBuilder.h"

namespace UAVCore {

    // --- 1. CẤU TRÚC GIAO TIẾP VỚI ENGINE BÊN NGOÀI ---
    struct EngineConfig {
        std::string unitFile;
        std::string vertexFile;
        std::string edgeFile;
        std::string targetFile;
        std::string unitsFolder;
        std::string uavPrefix = "data_uav_";
        std::string uavExt = ".csv";
    };

    // Chuẩn màu RGBA cho Unreal Engine (0.0f -> 1.0f)
    struct RGBA {
        float r, g, b, a;
    };

    // Quản lý trạng thái hiển thị của 1 Đơn vị máy bay
    struct UnitDisplayState {
        bool isVisible = true;
        RGBA lineColor = { 1.0f, 1.0f, 1.0f, 0.1f }; // Mặc định màu trắng đặc
    };

    // Gói dữ liệu Kết quả thuật toán để báo cáo lên Biểu đồ (Dashboard)
    struct MissionStatistics {
        double totalTargetValue = 0.0;
        double expectedDestroyedValue = 0.0;
        double ourLossCost = 0.0;
        int totalUAVDeployed = 0;
        int totalTargetsHit = 0;
        float successRate = 0.0f;

        // --- 2 DÒNG MỚI ĐỂ VẼ BIỂU ĐỒ ---
        std::vector<float> targetDamagePercents; // Lấp đầy % sát thương từng mục tiêu
        std::vector<std::string> targetNames;    // Tên các mục tiêu theo thứ tự
    };


    class UAVMissionEngine
    {
    public:
        UAVMissionEngine() = default;

        using LogCallback = std::function<void(const std::string&)>;
        void SetLogger(LogCallback logger);

        bool InitEngine(const EngineConfig& config);
        bool InitEngineFromDirectory(const std::string& dataDirectory);
        bool RunOptimization();

        // --- CÁC HÀM API HIỂN THỊ (GIAO DIỆN SẼ GỌI ĐẾN ĐÂY) ---

        // Cấp phát màu ngẫu nhiên/mặc định cho các đơn vị sau khi đọc dữ liệu
        void SetupDefaultDisplayStates();

        // Bật/tắt đường bay của một đơn vị cụ thể (UI Checkbox)
        void ToggleUnitVisibility(const std::string& unitId, bool isVisible);

        // Giao diện hỏi: Đơn vị này có được hiển thị không?
        bool IsUnitVisible(const std::string& unitId) const;

        // Giao diện hỏi: Đơn vị này dùng màu gì?
        RGBA GetUnitLineColor(const std::string& unitId) const;

        // Giao diện hỏi: Lấy dữ liệu để vẽ biểu đồ
        MissionStatistics GetMissionStatistics() const;


        // Truy xuất Core
        Graph& GetGraph() { return m_graph; }
        const AssignmentSolution& GetBestSolution() const { return m_bestSolution; }
        const OptimizationProblem& GetProblem() const { return m_problem; }

        void PrintAssignmentReport(std::ostream& os = std::cout) const;
        std::vector<std::string> GetUAVsAttackingTarget(int targetIndex) const;


    private:
        // Hàm nội bộ gọi đến Logger đã đăng ký
        void PrintLog(const std::string& msg);

        // Tự động giăng dây đường bay cho các căn cứ (Bạn mới thêm)
        void ConnectUnitsToGraph();

        Graph m_graph;
        OptimizationProblem m_problem;
        AssignmentSolution m_bestSolution;
        LogCallback m_logger = nullptr;

        // Lưu trữ trạng thái hiển thị của từng Đơn vị (Key = ID hoặc Tên Đơn vị)
        std::unordered_map<std::string, UnitDisplayState> m_unitDisplayStates;
    };

} // kết thúc namespace UAVCore