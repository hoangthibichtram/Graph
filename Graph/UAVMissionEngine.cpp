#include "UAVMissionEngine.h"
#include <iostream>
#include <vector>
#include <cmath> 

namespace UAVCore {

    void UAVMissionEngine::SetLogger(LogCallback logger)
    {
        m_logger = logger;
    }

    void UAVMissionEngine::PrintLog(const std::string& msg)
    {
        if (m_logger) m_logger(msg);
        else std::cout << msg << std::endl;
    }

    bool UAVMissionEngine::InitEngine(const EngineConfig& config)
    {
        PrintLog("[UAVMissionEngine] Bat dau doc du lieu MAP tu cac file CSV...");
        bool success = m_graph.readAllData(config.unitFile, config.vertexFile,
            config.edgeFile, config.targetFile, config.uavFile);
        if (success) {
            PrintLog("[UAVMissionEngine] TAI DU LIEU HOAN TAT THANH CONG.");
            SetupDefaultDisplayStates();
        }
        else PrintLog("[UAVMissionEngine] LOI nap file CSV!");

        return success;
    }

    bool UAVMissionEngine::InitEngineFromDirectory(const std::string& dataDirectory)
    {
        EngineConfig config;
        m_dataDir = dataDirectory;
        config.unitFile = dataDirectory + "\\UnitUAV.csv";
        config.vertexFile = dataDirectory + "\\Vertex.csv";
        config.edgeFile = dataDirectory + "\\Edge.csv";
        config.targetFile = dataDirectory + "\\Data_target.csv";
        config.uavFile = dataDirectory + "\\Data_UAV.csv";
        return InitEngine(config);
    }

    bool UAVMissionEngine::RunOptimization()
    {
        PrintLog("\n[UAVMissionEngine] ---> KHOI DONG MODULE TOI UU HOA...");
        m_problem = OptimizationBuilder::build(m_graph.getUnitList(), m_graph, m_dataDir);
        m_bestSolution = m_problem.bestSolution;
        PrintLog("[UAVMissionEngine] HOAN TAT TOI UU HOA.");
        return true;
    }

    // --- LOGIC HIỂN THỊ VÀ BIỂU ĐỒ (DÀNH CHO UI EXTERNAL) ---

    void UAVMissionEngine::SetupDefaultDisplayStates()
    {
        // Giả sử có sẵn các màu phân định cho các đội (Cấu trúc RGBA)
        std::vector<RGBA> defaultColors = {
            {1.0f, 0.0f, 0.0f, 0.5f}, // Đỏ (hơi trong suốt)
            {1.0f, 0.0f, 1.0f, 0.8f}, // Xanh lá
            {0.0f, 0.0f, 1.0f, 0.5f}, // Xanh lam
            {1.0f, 1.0f, 0.0f, 0.5f}, // Vàng 
            {0.5f, 0.0f, 0.5f, 0.5f}  //// Màu Tím
        };

        // Lấy số lượng Unit từ Graph, gán màu ngẫu nhiên hoặc theo thứ tự
        // (Lưu ý: Vì tôi chưa xem chi tiết UnitUAVList.h, mình tạm dùng index kiểu chuỗi)
        int unitCount = m_graph.getUnitList().getUnitCount();
        for (int i = 0; i < unitCount; ++i) {
            std::string unitName = "a" + std::to_string(i + 1); // Giả lập ID đội (Ví dụ: SQ1, SQ2)

            UnitDisplayState state;
            state.isVisible = true; // Mặc định bật tất cả
            state.lineColor = defaultColors[i % defaultColors.size()];

            m_unitDisplayStates[unitName] = state;
        }
        PrintLog("[UAVMissionEngine] Da khoi tao cau hinh hien thi (Mau sac/Visibility) cho cac Don vi.");
    }

    void UAVMissionEngine::ToggleUnitVisibility(const std::string& unitId, bool isVisible)
    {
        if (m_unitDisplayStates.find(unitId) != m_unitDisplayStates.end()) {
            m_unitDisplayStates[unitId].isVisible = isVisible;
        }
    }

    bool UAVMissionEngine::IsUnitVisible(const std::string& unitId) const
    {
        auto it = m_unitDisplayStates.find(unitId);
        if (it != m_unitDisplayStates.end()) return it->second.isVisible;
        return true; // Mặc định nếu không tìm thấy thì cho hiển thị
    }

    RGBA UAVMissionEngine::GetUnitLineColor(const std::string& unitId) const
    {
        auto it = m_unitDisplayStates.find(unitId);
        if (it != m_unitDisplayStates.end()) return it->second.lineColor;
        return { 1.0f, 1.0f, 1.0f, 1.0f }; // Mặc định trả về trắng
    }

    MissionStatistics UAVMissionEngine::GetMissionStatistics() const
    {
        MissionStatistics stats;
        auto targets = m_graph.GetTargets();

        stats.targetDamagePercents.resize(targets.size(), 0.0f);
        for (const auto& t : targets) {
            stats.totalTargetValue += t.value;
            stats.targetNames.push_back(t.name);
        }

        if (m_bestSolution.nUavTypes > 0 && !m_problem.uavs.empty()) {

            // Duyet theo prob.uavs (n UAV CA THE) chu khong theo danh sach don vi:
            // sau khi khai trien theo so luong, hai danh sach nay khong con cung kich thuoc.
            int n = m_bestSolution.nUavTypes;
            int m = m_bestSolution.nTargets;
            std::vector<double> targetMissProb(targets.size(), 1.0);

            for (int i = 0; i < n && i < (int)m_problem.uavs.size(); ++i) {
                bool deployed = false;
                for (int j = 0; j < m && j < (int)targets.size(); ++j) {
                    if (m_bestSolution.x[i * m + j] != 1) continue;
                    deployed = true;
                    targetMissProb[j] *= (1.0 - m_problem.uavs[i].pij[j]);
                    stats.ourLossCost += m_problem.uavs[i].cij[j];   // c_ij thuc te
                }
                if (deployed) stats.totalUAVDeployed++;
            }

            for (size_t j = 0; j < targets.size(); ++j) {
                double killProb = 1.0 - targetMissProb[j];
                stats.targetDamagePercents[j] = (float)(killProb * 100.0);
                stats.expectedDestroyedValue += (targets[j].value * killProb);

                // So muc tieu DA TIEN CONG = so muc tieu co y_j = 1, tuc la co it
                // nhat mot phuong tien duoc phan cong. Truoc day dem theo nguong
                // killProb > 0.5 - dai luong do khong co trong mo hinh.
                if (m_bestSolution.y((int)j) == 1) stats.totalTargetsHit++;
            }
        }

        if (stats.totalTargetValue > 0) {
            stats.successRate = (float)((stats.expectedDestroyedValue / stats.totalTargetValue) * 100.0);
        }

        return stats;
    }
    
    void UAVMissionEngine::PrintAssignmentReport(std::ostream& os) const
    {
        const auto& uavs = m_problem.uavs;
        const auto& targets = m_problem.targets;
        const auto& assignment = m_bestSolution;
        int n = assignment.nUavTypes, m = assignment.nTargets;

        for (int i = 0; i < n; ++i) {
            const UAVTypeOpt& uav = uavs[i];
            for (int j = 0; j < m; ++j) {
                if (assignment.x[i * m + j] == 1) {
                    os << "UAV " << uav.code
                        << " (Don vi: " << uav.unitName
                        << ") TAN CONG muc tieu: " << targets[j].name << std::endl;
                
                
                }
            }
        }
    }

    std::vector<std::string> UAVMissionEngine::GetUAVsAttackingTarget(int targetIndex) const
    {
        std::vector<std::string> result;
        int n = m_bestSolution.nUavTypes;
        int m = m_bestSolution.nTargets;
        if (targetIndex < 0 || targetIndex >= m) return result;

        for (int i = 0; i < n; ++i) {
            if (m_bestSolution.x[i * m + targetIndex] == 1) {
                // Lấy đúng code UAV từ bài toán tối ưu
                result.push_back(m_problem.uavs[i].code);
            }
        }
        return result;
    }


} 