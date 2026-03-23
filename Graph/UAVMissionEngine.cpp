#include "UAVMissionEngine.h"
#include <iostream>
#include <vector>
#include <cmath> // THÊM THƯ VIỆN NÀY ĐỂ TÍNHKHOẢNG CÁCH

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
        bool success = m_graph.readAllData(config.unitFile, config.vertexFile, config.edgeFile, config.targetFile, config.unitsFolder, config.uavPrefix, config.uavExt);

        if (success) {
            PrintLog("[UAVMissionEngine] TAI DU LIEU HOAN TAT THANH CONG.");
            
            // TỰ ĐỘNG GIĂNG DÂY TỪ CÁC CĂN CỨ VÀO MẠNG LƯỚI ĐIỂM
            ConnectUnitsToGraph();

            // Cài đặt trạng thái và màu sắc cơ bản ngay sau khi load
            SetupDefaultDisplayStates();
        }
        else PrintLog("[UAVMissionEngine] LOI nap file CSV!");

        return success;
    }

    bool UAVMissionEngine::InitEngineFromDirectory(const std::string& dataDirectory)
    {
        EngineConfig config;
        config.unitFile = dataDirectory + "\\UnitUAV.csv";
        config.vertexFile = dataDirectory + "\\Vertex.csv";
        config.edgeFile = dataDirectory + "\\Edge.csv";
        config.targetFile = dataDirectory + "\\Data_target.csv";
        config.unitsFolder = dataDirectory + "\\units";
        return InitEngine(config);
    }

    bool UAVMissionEngine::RunOptimization()
    {
        PrintLog("\n[UAVMissionEngine] ---> KHOI DONG MODULE TOI UU HOA...");
        m_problem = OptimizationBuilder::build(m_graph.getUnitList(), m_graph);
        m_bestSolution = m_problem.bestSolution;
        PrintLog("[UAVMissionEngine] HOAN TAT TOI UU HOA.");
        return true;
    }

    // --- LOGIC HIỂN THỊ VÀ BIỂU ĐỒ (DÀNH CHO UI EXTERNAL) ---

    void UAVMissionEngine::SetupDefaultDisplayStates()
    {
        // Giả sử có sẵn các màu phân định cho các đội (Cấu trúc RGBA)
        std::vector<RGBA> defaultColors = {
            {1.0f, 0.0f, 0.0f, 0.5f}, // Đỏ (hơi trong suốt)(SQ1)
            {1.0f, 0.0f, 1.0f, 0.8f}, // Xanh lá(SQ2)
            {0.0f, 0.0f, 1.0f, 0.5f}, // Xanh lam(SQ3)
            {1.0f, 1.0f, 0.0f, 0.5f}, // Vàng (SQ4)
            {0.5f, 0.0f, 0.5f, 0.5f}  //// Màu Tím cho Đội 5 (SQ5)
        };

        // Lấy số lượng Unit từ Graph, gán màu ngẫu nhiên hoặc theo thứ tự
        // (Lưu ý: Vì tôi chưa xem chi tiết UnitUAVList.h, mình tạm dùng index kiểu chuỗi)
        int unitCount = m_graph.getUnitList().getUnitCount();
        for (int i = 0; i < unitCount; ++i) {
            std::string unitName = "SQ" + std::to_string(i + 1); // Giả lập ID đội (Ví dụ: SQ1, SQ2)
            
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
            stats.totalTargetValue += t.value_usd;
            stats.targetNames.push_back(t.name);
        }

        if (m_bestSolution.nUavTypes > 0 && !m_bestSolution.paths.empty()) {
            
            std::vector<const UAV*> allUAVs;
            const auto& units = m_graph.getUnitList().getUnits();
            for (const auto& unit : units) {
                for (const auto& u : unit.getUAVs()) {
                    allUAVs.push_back(&u);
                }
            }

            std::vector<double> targetMissProb(targets.size(), 1.0);

            for (size_t i = 0; i < m_bestSolution.paths.size(); ++i) {
                const UAV* currentUav = (i < allUAVs.size()) ? allUAVs[i] : nullptr;
                bool isThisUAVDeployed = false; 
                double uavCost = (currentUav != nullptr) ? currentUav->getCostUsd() : 0.0; 

                const auto& targetsPathForUAVType = m_bestSolution.paths[i];
                for (size_t j = 0; j < targetsPathForUAVType.size(); ++j) {
                    const auto& path = targetsPathForUAVType[j];
                    if (path.size() >= 2) {
                        isThisUAVDeployed = true; 
                        
                        double pij = 0.0;
                        if (i < m_problem.uavs.size() && j < m_problem.uavs[i].pij.size()) {
                            pij = m_problem.uavs[i].pij[j];
                        }
                        targetMissProb[j] *= (1.0 - pij); 
                    }
                }
                
                if (isThisUAVDeployed) {
                    stats.totalUAVDeployed++;
                    stats.ourLossCost += uavCost;
                }
            }

            // ===== KIỂM ĐỊNH LẠI CÁC TRẠM PHÒNG KHÔNG ĐÃ SẬP CHƯA =====
            std::vector<int> activeDefenses;
            for (size_t j = 0; j < targets.size(); ++j) {
                double killProb = 1.0 - targetMissProb[j];
                // Phải so sánh đúng loại AirDefense
                if (targets[j].typeVertex == "AirDefense" && killProb < 0.6) {
                    activeDefenses.push_back((int)j);
                }
            }

            // === TÍNH TỔNG SÁT THƯƠNG (CÓ TRỪ HAO BỊ CHI VIỆN) ===
            for (size_t j = 0; j < targets.size(); ++j) {
                double killProb = 1.0 - targetMissProb[j];
                
                if (targets[j].typeVertex != "AirDefense") {
                    bool isProtected = false;
                    for (int defJ : activeDefenses) {
                        double dx = targets[j].x - targets[defJ].x;
                        double dy = targets[j].y - targets[defJ].y;
                        if (std::sqrt(dx*dx + dy*dy) <= 50.0) { // Bán kính radar 50
                            isProtected = true; break;
                        }
                    }
                    if (isProtected) killProb *= 0.5; // Giảm 50% sát thương nếu bị yểm trợ!
                }

                stats.targetDamagePercents[j] = (float)(killProb * 100.0);
                stats.expectedDestroyedValue += (targets[j].value_usd * killProb);
                
                if (killProb > 0.5) stats.totalTargetsHit++;
            }
        }

        if (stats.totalTargetValue > 0) {
            stats.successRate = (float)((stats.expectedDestroyedValue / stats.totalTargetValue) * 100.0);
        }

        return stats;
    }

    // =========================================================================
    // HÀM TỰ ĐỘNG CẮM ĐƯỜNG BAY CHO ĐƠN VỊ VÀO BẢN ĐỒ
    void UAVMissionEngine::ConnectUnitsToGraph()
    {
        // Ta sẽ coi mỗi Đơn vị như là 1 trạm Vertex ảo, đặt ID bắt đầu từ 10000 
        // để không bị trùng lặp với các điểm point số 1, 2, 3... trong CSV có sẵn
        int unitBaseId = 10000; 

        for (const auto& unit : m_graph.getUnitList().getUnits())
        {
            // 1. Tìm xem cái đỉnh nào của bản đồ đang nằm gần căn cứ này nhất
            int nearestVId = m_graph.findNearestVertex(unit.getX(), unit.getY());
            
            double connectRadius = 100.0; // Bán kính kết nối mặc định

            if (nearestVId != -1) {
                // 2. Tính khoảng cách từ căn cứ Unit tới điểm gần nhất
                Vertex nearestV = m_graph.GetVertexById(nearestVId);
                double dx = unit.getX() - nearestV.x;
                double dy = unit.getY() - nearestV.y;
                double distToNearest = std::sqrt(dx * dx + dy * dy);
                
                // Cài đặt bán kính kết nối bao trùm ít nhất 1.5 lần khoảng cách 
                // để luôn gắn được ít nhất vài đường dây vào các trạm xung quanh
                if ((distToNearest * 1.5) > connectRadius) {
                    connectRadius = distToNearest * 1.5;
                }
            }

            // 3. Phép thuật ở đây: Chúng ta gọi AddVertex của Graph.
            // Hàm AddVertex của bạn tự động có chức năng tạo Edge nối tới mọi điểm lân cận trong bán kính Radius!
            m_graph.AddVertex(unitBaseId, unit.getX(), unit.getY(), 0.0, static_cast<float>(connectRadius), nullptr, false);
            
            unitBaseId++; // Tăng ID cho Đơn vị tiếp theo
        }
        
        PrintLog("[UAVMissionEngine] Da giang day duong bay ket noi cac Base (Unit) vao Mang luoi!");
    }
    // =========================================================================

} // kết thúc namespace UAVCore