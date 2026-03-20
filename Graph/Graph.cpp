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

    static inline bool IsNumberStart(const std::string& s)
    {
        if (s.empty()) return false;
        char c = s.front();
        return (c >= '0' && c <= '9') || c == '-' || c == '+';
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

bool Graph::AddVertex(int id, double x, double y, double z, float radius, int* outConnections, bool verbose)
{
    if (outConnections) *outConnections = 0;
    if (idIndexMap_.find(id) != idIndexMap_.end())
    {
        if (verbose) std::cout << "Vertex with id " << id << " already exists. Skipping.\n";
        return false;
    }

    std::size_t before = edges_.size();
    vertices_.emplace_back(id, x, y, z);
    idIndexMap_[id] = vertices_.size() - 1;

    for (std::size_t i = 0; i + 1 < vertices_.size(); ++i)
    {
        const Vertex& other = vertices_[i];
        double dist = vertices_.back().distanceTo(other);
        if (dist <= static_cast<double>(radius))
        {
            AddEdge(vertices_.back(), other, static_cast<float>(dist));
            AddEdge(other, vertices_.back(), static_cast<float>(dist));
        }
    }

    int added = static_cast<int>(edges_.size() - before);
    if (outConnections) *outConnections = added;
    if (verbose) std::cout << "Added " << added << " connections for vertex id=" << id << ".\n";
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

std::vector<Vertex> Graph::findVerticesInRadius(double x, double y, double z, double radius) const
{
    std::vector<Vertex> result;
    for (const auto& v : vertices_)
    {
        double dist = std::sqrt((v.x - x) * (v.x - x) + (v.y - y) * (v.y - y) + (v.z - z) * (v.z - z));
        if (dist <= radius) result.push_back(v);
    }
    return result;
}

// --- Specific CSV readers ---

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
                    continue; // Skip if vertex not found
                }

                const Vertex& sV = vertices_[idIndexMap_[startId]];
                const Vertex& eV = vertices_[idIndexMap_[endId]];

                float w = static_cast<float>(ComputeDistance(sV, eV));
                AddEdge(sV, eV, w);
            }
        }
        catch (...) { continue; }
    }
    return true;
}

bool Graph::ReadTargetFile(const std::string& path)
{
    std::ifstream ifs(path);
    ifs >> std::noskipws;

    if (!ifs.is_open()) return false;
    std::string header;
    if (!std::getline(ifs, header)) return false;

    RemoveUTF8BOM(header);
    header = Trim(header);
    char delim = DetectDelimiter(header);
    auto hdr = ParseCsvLine(header, delim);
    std::unordered_map<std::string, size_t> hidx;
    for (size_t i = 0; i < hdr.size(); ++i) hidx[ToLower(hdr[i])] = i;

    auto get = [&](std::initializer_list<const char*> names)->int {
        for (auto n : names) {
            auto it = hidx.find(ToLower(n));
            if (it != hidx.end()) return static_cast<int>(it->second);
        }
        return -1;
        };

    int idxX = get({ "x" });
    int idxY = get({ "y" });
    int idxTargetId = get({ "target_id","id","targetid" });
    int idxCode = get({ "code" });
    int idxName = get({ "name" });
    int idxAlt = get({ "altitude","alt" });
    int idxPriority = get({ "priority" });
    int idxExpl = get({ "explosize","explosive","expl" });
    int idxValue = get({ "value_usd","value","price" });
    int idxIdVertex = get({ "id_vertex","idvertex","vertex_id" });
    int idxTypeVertex = get({ "typevertex","type_vertex" });

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
            Target t{};
            if (idxTargetId >= 0 && idxTargetId < static_cast<int>(tok.size())) t.target_id = std::stoi(tok[idxTargetId]);
            if (idxCode >= 0 && idxCode < static_cast<int>(tok.size())) t.code = tok[idxCode];
            if (idxName >= 0 && idxName < static_cast<int>(tok.size())) t.name = tok[idxName];
            if (idxX >= 0 && idxX < static_cast<int>(tok.size())) t.x = std::stod(tok[idxX]);
            if (idxY >= 0 && idxY < static_cast<int>(tok.size())) t.y = std::stod(tok[idxY]);
            if (idxAlt >= 0 && idxAlt < static_cast<int>(tok.size())) t.altitude = std::stof(tok[idxAlt]);
            if (idxPriority >= 0 && idxPriority < static_cast<int>(tok.size())) t.priority = std::stoi(tok[idxPriority]);
            if (idxExpl >= 0 && idxExpl < static_cast<int>(tok.size())) t.explosize = std::stof(tok[idxExpl]);
            if (idxValue >= 0 && idxValue < static_cast<int>(tok.size())) t.value_usd = std::stod(tok[idxValue]);
            if (idxIdVertex >= 0 && idxIdVertex < static_cast<int>(tok.size())) t.id_vertex = std::stoi(tok[idxIdVertex]);
            if (idxTypeVertex >= 0 && idxTypeVertex < static_cast<int>(tok.size())) t.typeVertex = tok[idxTypeVertex];

            targets_.push_back(std::move(t));
        }
        catch (...) { continue; }
    }
    return true;
}

bool Graph::ReadFromFile(const std::string& path)
{
    std::string lower = ToLower(path);
    if (lower.find("vertex") != std::string::npos) return ReadVerticesFile(path);
    if (lower.find("edge") != std::string::npos) return ReadEdgesFile(path);
    if (lower.find("target") != std::string::npos) return ReadTargetFile(path);

    if (ReadVerticesFile(path)) return true;
    if (ReadEdgesFile(path)) return true;
    return false;
}

bool Graph::readAllData(const std::string& unitFile,
    const std::string& vertexFile,
    const std::string& edgeFile,
    const std::string& targetFile,
    const std::string& unitsFolder,
    const std::string& uavPrefix,
    const std::string& uavExt)
{
    bool success = true;

    if (!ReadVerticesFile(vertexFile)) success = false;
    if (!ReadEdgesFile(edgeFile)) success = false;
    if (!unitList.loadUnitsFromFile(unitFile)) success = false;
    unitList.loadUAVsFromPerUnitFiles(unitsFolder, uavPrefix, uavExt);
    if (!ReadTargetFile(targetFile)) success = false;

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

int Graph::findNearestVertex(double x, double y) const
{
    double best = 1e18;
    int bestId = -1;

    for (const auto& v : vertices_) {
        double dx = v.x - x;
        double dy = v.y - y;
        double d = dx * dx + dy * dy;
        if (d < best) {
            best = d;
            bestId = v.id;
        }
    }
    return bestId;
}

void Graph::removeVertexZero()
{
    auto it = idIndexMap_.find(0);
    if (it != idIndexMap_.end()) {
        int index = it->second;
        vertices_.erase(vertices_.begin() + index);
        idIndexMap_.erase(it);

        for (size_t i = 0; i < vertices_.size(); i++) {
            idIndexMap_[vertices_[i].id] = static_cast<int>(i);
        }
    }
}