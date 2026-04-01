#pragma once
#include "UnitUAVList.h"
#include "resource.h"

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <cstdint>

struct Vertex
{
    int id;
    double x;
    double y;
    double z;
    std::string type; // optional, from CSV typeVertex

    Vertex() noexcept
        : id(-1), x(0.0), y(0.0), z(0.0), type()
    {
    }

    Vertex(int id_, double x_, double y_, double z_, std::string type_ = {}) noexcept
        : id(id_), x(x_), y(y_), z(z_), type(std::move(type_))
    {
    }

    double distanceTo(const Vertex& other) const noexcept
    {
        const double dx = x - other.x;
        const double dy = y - other.y;
        const double dz = z - other.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    bool operator==(const Vertex& other) const noexcept
    {
        return id == other.id;
    }
};

struct Edge
{
    Vertex start;
    Vertex end;
    float weight;

    Edge() noexcept
        : start(), end(), weight(0.0f)
    {
    }

    Edge(const Vertex& s, const Vertex& e, float w) noexcept
        : start(s), end(e), weight(w)
    {
    }
};

struct Target
{
    int target_id;
    std::string code;
    std::string name;
    double x;
    double y;
    float altitude;
    float explosize;
    double value_usd;
    int id_vertex;
    std::string typeVertex;
};

class Graph
{
public:
    Graph() = default;
    void removeVertexZero();
    // Generic CSV read (kept for compatibility)
    bool ReadFromFile(const std::string& path);

    // Specific readers requested
    bool ReadVerticesFile(const std::string& path);   // x,y,id,typeVertex
    bool ReadEdgesFile(const std::string& path);      // WKT,id,start,end,weight
    //bool ReadUavFile(const std::string& path);        // Data_uav columns specified
    bool ReadTargetFile(const std::string& path);     // Data_target columns specified

    // Hàm đọc tất cả dữ liệu
    bool readAllData(const std::string& unitFile,
        const std::string& vertexFile,
        const std::string& edgeFile,
        const std::string& targetFile,
        const std::string& unitsFolder,
        const std::string& uavPrefix = "data_uav_",
        const std::string& uavExt = ".csv");

    int findNearestVertex(double x, double y) const;
    double shortestPathDistance(int startId, int endId) const;
    std::vector<int> shortestPath(int startId, int endId) const;

    // Getter cho unitList
    UnitUAVList& getUnitList() { return unitList; }
    const UnitUAVList& getUnitList() const { return unitList; }

    // Add a vertex with the given id and coordinates.
    bool AddVertex(int id, double x, double y, double z, float radius, int* outConnections = nullptr, bool verbose = false);

    // Accessors
    const std::vector<Vertex>& GetVertices() const noexcept { return vertices_; }
    const std::vector<Edge>& GetEdges() const noexcept { return edges_; }

    Vertex GetVertexById(int id) const;

    const std::vector<Target>& GetTargets() const noexcept { return targets_; }
    int getVertexCount() const noexcept { return static_cast<int>(vertices_.size()); }
    int getEdgeCount() const noexcept { return static_cast<int>(edges_.size()); }

    // Find vertex by id. Returns pointer to vertex in internal vector or nullptr if not found.
    Vertex* findVertexById(int id) noexcept;

    // Find vertices within radius of point (x,y,z). Returns copies of matching vertices.
    std::vector<Vertex> findVerticesInRadius(double x, double y, double z, double radius) const;
    //hàm tính khoảng cách giữa 2 vertex (dùng cho kiểm tra khi thêm vertex mới)
    static double ComputeDistance(const Vertex& a, const Vertex& b)
    {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }


private:
    bool AddEdge(const Vertex& start, const Vertex& end, float weight);

    static inline uint64_t MakeEdgeKey(int from, int to) noexcept
    {
        return (static_cast<uint64_t>(static_cast<uint32_t>(from)) << 32)
            | static_cast<uint32_t>(to);
    }

    // BỔ SUNG: Quản lý đơn vị
    UnitUAVList unitList;

    std::vector<Vertex> vertices_;
    std::vector<Edge> edges_;
    std::vector<Vertex>& GetVerticesRef() { return vertices_; }
    std::vector<Target>& GetTargetsRef() { return targets_; }
    std::vector<Target> targets_;
    std::unordered_map<int, std::size_t> idIndexMap_; // maps vertex id -> index in vertices_
    std::unordered_set<uint64_t> edgeSet_; // track existing directed edges
};