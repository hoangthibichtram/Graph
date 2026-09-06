#include "Graph.h"
#include <queue>
#include <utility>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <iostream>

namespace
{
    static inline std::string Trim(const std::string& s)
    {
        if (s.empty()) return s;

        size_t start = 0;
        while (start < s.size() &&
            std::isspace((unsigned char)s[start]))
            start++;

        size_t end = s.size();
        while (end > start &&
            std::isspace((unsigned char)s[end - 1]))
            end--;

        return s.substr(start, end - start);
    }

    static inline std::string ToLower(std::string s)
    {
        for (char& ch : s)
            ch = std::tolower((unsigned char)ch);
        return s;
    }

    static inline void RemoveUTF8BOM(std::string& s)
    {
        if (s.size() >= 3 &&
            (unsigned char)s[0] == 0xEF &&
            (unsigned char)s[1] == 0xBB &&
            (unsigned char)s[2] == 0xBF)
        {
            s.erase(0, 3);
        }
    }

    // Parse CSV line with simple quoted field handling
    static inline std::vector<std::string> ParseCsvLine(const std::string& line, char delim)
    {
        std::vector<std::string> out;
        std::string cur;
        bool inQuotes = false;
        for (size_t i = 0; i < line.size(); ++i)
        {
            char c = line[i];
            if (c == '"')
            {
                if (inQuotes && i + 1 < line.size() && line[i + 1] == '"')
                {
                    cur.push_back('"'); ++i;
                }
                else
                {
                    inQuotes = !inQuotes;
                }
            }
            else if (!inQuotes && c == delim)
            {
                out.push_back(Trim(cur)); cur.clear();
            }
            else
            {
                cur.push_back(c);
            }
        }
        out.push_back(Trim(cur));
        return out;
    }

    // helper to detect delimiter (comma vs semicolon)
    static inline char DetectDelimiter(const std::string& sample)
    {
        size_t c = std::count(sample.begin(), sample.end(), ',');
        size_t s = std::count(sample.begin(), sample.end(), ';');
        return (s > c) ? ';' : ',';
    }
}

// --- Core graph functions ---

bool Graph::AddEdge(const Vertex& start, const Vertex& end, float weight)
{
    uint64_t key = MakeEdgeKey(start.id, end.id);
    if (edgeSet_.find(key) != edgeSet_.end()) return false;
    edges_.emplace_back(start, end, weight);
    edgeSet_.insert(key);
    return true;
}

Vertex* Graph::findVertexById(int id) noexcept
{
    auto it = idIndexMap_.find(id);
    if (it == idIndexMap_.end()) return nullptr;
    std::size_t idx = it->second;
    if (idx >= vertices_.size()) return nullptr;
    return &vertices_[idx];
}

bool Graph::ReadVerticesFile(const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    std::string header;
    if (!std::getline(ifs, header)) return false;
    RemoveUTF8BOM(header);
    header = Trim(header);
    char delim = DetectDelimiter(header);
    auto hdr = ParseCsvLine(header, delim);
    std::unordered_map<std::string, size_t> hidx;
    for (size_t i = 0; i < hdr.size(); ++i) hidx[ToLower(hdr[i])] = i;

    int idxX = (hidx.count("x") ? static_cast<int>(hidx["x"]) : (hidx.count("lon") ? static_cast<int>(hidx["lon"]) : -1));
    int idxY = (hidx.count("y") ? static_cast<int>(hidx["y"]) : (hidx.count("lat") ? static_cast<int>(hidx["lat"]) : -1));
    int idxId = (hidx.count("id") ? static_cast<int>(hidx["id"]) : -1);
    int idxType = (hidx.count("typevertex") ? static_cast<int>(hidx["typevertex"]) : -1);

    std::string line;
    while (std::getline(ifs, line))
    {
        RemoveUTF8BOM(line);
        line = Trim(line);
        if (line.empty()) continue;
        if (line[0] == '#' || line[0] == '/') continue;
        auto tok = ParseCsvLine(line, delim);
        try
        {
            if (idxId >= 0 && idxX >= 0 && idxY >= 0 &&
                idxId < static_cast<int>(tok.size()) && idxX < static_cast<int>(tok.size()) && idxY < static_cast<int>(tok.size()))
            {
                int id = std::stoi(tok[idxId]);
                double x = std::stod(tok[idxX]);
                double y = std::stod(tok[idxY]);
                double z = 0.0;
                std::string type;
                if (idxType >= 0 && idxType < static_cast<int>(tok.size())) type = tok[idxType];
                if (idIndexMap_.find(id) == idIndexMap_.end())
                {
                    vertices_.push_back(Vertex(id, x, y, z, type));
                    idIndexMap_[id] = vertices_.size() - 1;
                }
            }
        }
        catch (...) { continue; }
    }
    return true;
}

bool Graph::ReadEdgesFile(const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    std::string header;
    if (!std::getline(ifs, header)) return false;
    RemoveUTF8BOM(header);
    header = Trim(header);
    char delim = DetectDelimiter(header);
    auto hdr = ParseCsvLine(header, delim);
    std::unordered_map<std::string, size_t> hidx;
    for (size_t i = 0; i < hdr.size(); ++i) {
        hidx[ToLower(hdr[i])] = i;
    }
    int idxStart = (hidx.count("start") ? static_cast<int>(hidx["start"]) : (hidx.count("from") ? static_cast<int>(hidx["from"]) : -1));
    int idxEnd = (hidx.count("end") ? static_cast<int>(hidx["end"]) : (hidx.count("to") ? static_cast<int>(hidx["to"]) : -1));
    int idxWeight = (hidx.count("weight") ? static_cast<int>(hidx["weight"]) : -1);
    int idxId = (hidx.count("id") ? static_cast<int>(hidx["id"]) : -1);

    std::string line;
    while (std::getline(ifs, line))
    {
        RemoveUTF8BOM(line);
        line = Trim(line);
        if (line.empty()) continue;

        if (line[0] == '#' || line[0] == '/') continue;
        auto tok = ParseCsvLine(line, delim);
        try
        {
            if (idxStart >= 0 && idxEnd >= 0 &&
                idxStart < static_cast<int>(tok.size()) &&
                idxEnd < static_cast<int>(tok.size()))
            {
                int startId = std::stoi(tok[idxStart]);
                int endId = std::stoi(tok[idxEnd]);

                if (startId == 0 || endId == 0) continue;

                if (idIndexMap_.find(startId) == idIndexMap_.end() ||
                    idIndexMap_.find(endId) == idIndexMap_.end())
                {
                    continue;
                }

                const Vertex& sV = vertices_[idIndexMap_[startId]];
                const Vertex& eV = vertices_[idIndexMap_[endId]];

                float w = static_cast<float>(ComputeDistance(sV, eV));
                AddEdge(sV, eV, w);
                AddEdge(eV, sV, w);// cạnh 2 chiều.
            }
        }
        catch (...) { continue; }
    }
    // Duyệt tìm những đỉnh nào (như đỉnh 31) đang KHÔNG có bất kỳ Edge nào nối tới nó
    for (const auto& v : vertices_) {
        bool hasConnection = false;

        // Kiểm tra xem có edge nào dính dáng tới v.id hay không
        for (const auto& e : edges_) {
            if (e.start.id == v.id || e.end.id == v.id) {
                hasConnection = true;
                break;
            }
        }

        // Nếu phát hiện đỉnh "cô đơn" (chưa được nối từ Edge.csv)
        if (!hasConnection) {
            // Tìm 2 đỉnh gần nó nhất để cắm dây
            int nearest1 = -1, nearest2 = -1;
            double minDist1 = 1e18, minDist2 = 1e18;

            for (const auto& other : vertices_) {
                if (other.id == v.id) continue;

                double dx = v.x - other.x;
                double dy = v.y - other.y;
                double dist = std::sqrt(dx * dx + dy * dy);

                if (dist < minDist1) {
                    minDist2 = minDist1; nearest2 = nearest1;
                    minDist1 = dist; nearest1 = other.id;
                }
                else if (dist < minDist2) {
                    minDist2 = dist; nearest2 = other.id;
                }
            }

            // Tiến hành nối cáp
            if (nearest1 != -1) {
                AddEdge(v, vertices_[idIndexMap_[nearest1]], static_cast<float>(minDist1));
                AddEdge(vertices_[idIndexMap_[nearest1]], v, static_cast<float>(minDist1)); // Nối 2 chiều
            }
            if (nearest2 != -1) {
                AddEdge(v, vertices_[idIndexMap_[nearest2]], static_cast<float>(minDist2));
                AddEdge(vertices_[idIndexMap_[nearest2]], v, static_cast<float>(minDist2));
            }
            std::cout << "[Graph] Da tu dong ket noi mien nhiem cho Dinh: " << v.id << "\n";
        }
    }
    // ---------------------------------------------------------------------

    return true;
}

bool Graph::ReadTargetFile(const std::string& path)
{
    std::ifstream ifs(path);
    ifs >> std::noskipws;

    if (!ifs.is_open()) {
        std::cout << "[LỖI LỚN] KHÔNG THỂ MỞ FILE: " << path << "\n";
        return false;
    }

    std::string header;
    if (!std::getline(ifs, header)) return false;

    char delim = DetectDelimiter(header);
    RemoveUTF8BOM(header);
    auto hdr = ParseCsvLine(header, delim);
    std::unordered_map<std::string, size_t> hidx;
    for (size_t i = 0; i < hdr.size(); ++i) hidx[ToLower(hdr[i])] = i;

    auto col = [&](std::initializer_list<const char*> names)->int {
        for (auto n : names) { auto it = hidx.find(ToLower(n)); if (it != hidx.end()) return (int)it->second; }
        return -1;
        };
    int iX = col({ "x" });
    int iY = col({ "y" });
    int iTid = col({ "target_id","id" });
    int iCode = col({ "code" });
    int iName = col({ "name" });
    int iPrio = col({ "priority" });
    int iExp = col({ "explosive" });
    int iVal = col({ "military_value" });
    int iVtx = col({ "id_vertex","vertexid" });

    std::string line;
    while (std::getline(ifs, line))
    {
        RemoveUTF8BOM(line);
        line = Trim(line);
        if (line.empty()) continue;
        if (line[0] == '#' || line[0] == '/') continue;

        auto tok = ParseCsvLine(line, delim);

        try
        {
            if (tok.size() >= 9)
            {
                Target t{};
                auto get = [&](int idx)->std::string {
                    return (idx >= 0 && idx < (int)tok.size()) ? tok[idx] : std::string();
                    };

                t.x = get(iX).empty() ? 0.0 : std::stod(get(iX));
                t.y = get(iY).empty() ? 0.0 : std::stod(get(iY));
                t.target_id = get(iTid).empty() ? 0 : std::stoi(get(iTid));
                t.code = Trim(get(iCode));
                t.name = Trim(get(iName));
                t.explosive = get(iExp).empty() ? 0.0f : std::stof(get(iExp));
                t.value = get(iVal).empty() ? 0.0 : std::stod(get(iVal)); 
                t.id_vertex = get(iVtx).empty() ? 0 : std::stoi(get(iVtx));
                t.priority = get(iPrio).empty() ? 1 : std::stoi(get(iPrio));

                // Lấy tọa độ thật từ đỉnh (giữ nguyên như cũ)
                Vertex* vNode = findVertexById(t.id_vertex);
                if (vNode != nullptr) {
                    t.x = vNode->x;
                    t.y = vNode->y;
                }
                targets_.push_back(std::move(t));
                std::cout << "[TARGET THANH CONG] Bat duoc: " << t.name << " dinh: " << t.id_vertex << "\n";
            }
            else {
                std::cout << "[MẤT TARGET] Dòng nay qua ngan (chua toi 9 cot): " << line << "\n";
            }
        }
        catch (...) {
            std::cout << "[BĂM CSV LỖI] Lỗi format chuỗi thành số tại dòng: " << line << "\n";
        }
    }
    return true;
}

bool Graph::readAllData(const std::string& unitFile,
    const std::string& vertexFile,
    const std::string& edgeFile,
    const std::string& targetFile,
    const std::string& uavFile)
{
    bool success = true;

    if (!ReadVerticesFile(vertexFile)) success = false;
    if (!ReadEdgesFile(edgeFile))      success = false;
    if (!unitList.loadUnitsFromFile(unitFile)) success = false;
    if (!unitList.loadUAVsFromCombinedFile(uavFile)) success = false;
    if (!ReadTargetFile(targetFile))   success = false;
    return success;
}

Vertex Graph::GetVertexById(int id) const {
    auto it = idIndexMap_.find(id);
    if (it == idIndexMap_.end()) return Vertex();
    return vertices_[it->second];
}

double Graph::shortestPathDistance(int startId, int endId) const
{
    const double INF = 1e18;
    std::unordered_map<int, double> dist;
    for (auto& v : vertices_) dist[v.id] = INF;

    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

    dist[startId] = 0;
    pq.push({ 0, startId });

    while (!pq.empty()) {
        auto top = pq.top();
        double d = top.first;
        int u = top.second;
        pq.pop();
        if (d != dist[u]) continue;
        if (u == endId) break;

        for (auto& e : edges_) {
            if (e.start.id != u) continue;
            int v = e.end.id;
            double nd = d + e.weight;
            if (nd < dist[v]) {
                dist[v] = nd;
                pq.push({ nd, v });
            }
        }
    }
    return dist[endId];
}

std::vector<int> Graph::shortestPath(int startId, int endId) const
{
    const double INF = 1e18;
    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> prev;

    for (auto& v : vertices_) dist[v.id] = INF;

    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

    dist[startId] = 0;
    pq.push(P(0, startId));

    while (!pq.empty()) {
        P top = pq.top(); pq.pop();
        double d = top.first;
        int u = top.second;

        if (d != dist[u]) continue;
        if (u == endId) break;

        for (auto& e : edges_) {
            if (e.start.id != u) continue;
            int v = e.end.id;
            double nd = d + e.weight;
            if (nd < dist[v]) {
                dist[v] = nd;
                prev[v] = u;
                pq.push(P(nd, v));
            }
        }
    }

    std::vector<int> path;
    if (!prev.count(endId)) return path;

    for (int v = endId; v != startId; v = prev[v])
        path.push_back(v);
    path.push_back(startId);
    std::reverse(path.begin(), path.end());
    return path;
}
