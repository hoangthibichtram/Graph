#include "UAVMissionEngine.h"
#include <iostream>

namespace UAVCore {

    void UAVMissionEngine::SetLogger(LogCallback logger)
    {
        m_logger = logger;
    }

    void UAVMissionEngine::PrintLog(const std::string& msg)
    {
        if (m_logger) {
            m_logger(msg); // Nếu bên ngoài có đăng ký hàm Log, thì vứt ra ngoài.
        }
        else {
            // Fallback: Nếu không đăng ký gì, mặc định in ra std::cout
            std::cout << msg << std::endl;
        }
    }

    bool UAVMissionEngine::InitEngine(const EngineConfig& config)
    {
        PrintLog("[UAVMissionEngine] Bat dau doc du lieu MAP tu cac file CSV...");

        bool success = m_graph.readAllData(
            config.unitFile,
            config.vertexFile,
            config.edgeFile,
            config.targetFile,
            config.unitsFolder,
            config.uavPrefix,
            config.uavExt
        );

        if (success) {
            PrintLog("[UAVMissionEngine] TAI DU LIEU HOAN TAT THANH CONG.");
        }
        else {
            PrintLog("[UAVMissionEngine] LOI: Xay ra loi trong qua trinh nap file CSV!");
        }
        return success;
    }

    bool UAVMissionEngine::InitEngineFromDirectory(const std::string& dataDirectory)
    {
        PrintLog("[UAVMissionEngine] Phant tich duong dan tu thu muc: " + dataDirectory);

        // Tự động lắp ráp cấu trúc file từ thư mục
        EngineConfig config;
        config.unitFile    = dataDirectory + "\\UnitUAV.csv";
        config.vertexFile  = dataDirectory + "\\Vertex.csv";
        config.edgeFile    = dataDirectory + "\\Edge.csv";
        config.targetFile  = dataDirectory + "\\Data_target.csv";
        config.unitsFolder = dataDirectory + "\\units";

        return InitEngine(config);
    }

    bool UAVMissionEngine::RunOptimization()
    {
        PrintLog("\n[UAVMissionEngine] ---> KHOI DONG MODULE TOI UU HOA...");

        // Build cấu trúc thuật toán
        m_problem = OptimizationBuilder::build(m_graph.getUnitList(), m_graph);

        // Thuật toán gán kết quả tốt nhất ở đây
        m_bestSolution = m_problem.bestSolution;

        PrintLog("[UAVMissionEngine] HOAN TAT TOI UU HOA.");
        return true;
    }

} // kết thúc namespace UAVCore