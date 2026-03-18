// Graph.cpp : Updated: new CSV readers for Vertex, Edge, Data_uav, Data_target.
// Uses robust header mapping for specified column names.
#include "GraphRenderer.h"
#include "framework.h"
#include "Graph.h"
#include "OptimizationBuilder.h"
#include "OptimizationProblem.h"
#include "UAVOptimization.h"
#include <queue>
#include <utility>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <iostream>

#define MAX_LOADSTRING 100

// Global Variables:
Graph g_graph;
GraphRenderer g_renderer;
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING]; 
HWND hWnd;

// Forward declarations:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

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

// --- Core graph functions (unchanged behavior) ---

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

// --- New specific CSV readers ---

// Read Vertex.csv with columns: x,y,id,typeVertex
bool Graph::ReadVerticesFile(const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    // read header
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

// Read Edge.csv with columns: WKT,id,start,end,weight
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
        std::cout << "HDR[" << i << "] = '" << hdr[i] << "'\n";
        hidx[ToLower(hdr[i])] = i;
    }
    int idxStart = (hidx.count("start") ? static_cast<int>(hidx["start"]) : (hidx.count("from") ? static_cast<int>(hidx["from"]) : -1));
    int idxEnd = (hidx.count("end") ? static_cast<int>(hidx["end"]) : (hidx.count("to") ? static_cast<int>(hidx["to"]) : -1));
    int idxWeight = (hidx.count("weight") ? static_cast<int>(hidx["weight"]) : -1);
    int idxId = (hidx.count("id") ? static_cast<int>(hidx["id"]) : -1);

    std::cout << "idxStart=" << idxStart
        << " idxEnd=" << idxEnd
        << " idxWeight=" << idxWeight << "\n";

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

                if (startId == 0 || endId == 0) {
                    std::cout << "SKIP EDGE with ID 0: " << startId << " -> " << endId << std::endl;
                    continue;
                }
                // Kiểm tra vertex tồn tại
                if (idIndexMap_.find(startId) == idIndexMap_.end() ||
                    idIndexMap_.find(endId) == idIndexMap_.end())
                {
                    std::cout << "SKIP EDGE: " << startId << " -> " << endId
                        << " (vertex not found)\n";
                    continue;
                }

                // Lấy vertex thật
                const Vertex& sV = vertices_[idIndexMap_[startId]];
                const Vertex& eV = vertices_[idIndexMap_[endId]];

                // ⭐ TÍNH WEIGHT THEO KHOẢNG CÁCH THỰC
                float w = static_cast<float>(ComputeDistance(sV, eV));

                // Thêm cạnh
                AddEdge(sV, eV, w);
            }

        }
        catch (...) { continue; }
    }
    std::cout << "DONE EDGE FILE\n";
    std::cout << "Total edges added = " << edges_.size() << "\n";

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

// Keep existing generic ReadFromFile for compatibility (delegates by file name heuristic)
bool Graph::ReadFromFile(const std::string& path)
{
    // simple heuristic by filename
    std::string lower = ToLower(path);
    if (lower.find("vertex") != std::string::npos) return ReadVerticesFile(path);
    if (lower.find("edge") != std::string::npos) return ReadEdgesFile(path);
    //if (lower.find("uav") != std::string::npos) return ReadUavFile(path);
    if (lower.find("target") != std::string::npos) return ReadTargetFile(path);

    // fallback: try edge read then vertex read
    if (ReadVerticesFile(path)) return true;
    if (ReadEdgesFile(path)) return true;
    return false;
}

// Thêm vào cuối file Graph.cpp, trước phần Win32

bool Graph::readAllData(const std::string& unitFile,
    const std::string& vertexFile,
    const std::string& edgeFile,
    const std::string& unitsFolder,
    const std::string& uavPrefix,
    const std::string& uavExt)
{
    bool success = true;

    std::cout << "\n========== BẮT ĐẦU ĐỌC DỮ LIỆU ==========\n";

    //  Đọc vertices
    std::cout << "1. Đọc vertices từ " << vertexFile << "..." << std::endl;
    if (!ReadVerticesFile(vertexFile)) {
        std::cerr << "    Lỗi đọc file vertex!" << std::endl;
        success = false;
    }
    else {
        std::cout << "    Đã đọc " << vertices_.size() << " vertices" << std::endl;
    }

    //  Đọc edges
    std::cout << "2. Đọc edges từ " << edgeFile << "..." << std::endl;
    if (!ReadEdgesFile(edgeFile)) {
        std::cerr << "    Lỗi đọc file edge!" << std::endl;
        success = false;
    }
    else {
        std::cout << "    Đã đọc " << edges_.size() << " edges" << std::endl;
    }

    //  Đọc đơn vị
    std::cout << "3. Đọc đơn vị từ " << unitFile << "..." << std::endl;
    if (!unitList.loadUnitsFromFile(unitFile)) {
        std::cerr << "    Lỗi đọc file đơn vị!" << std::endl;
        success = false;
    }
    else {
        std::cout << "    Đã đọc " << unitList.getUnitCount() << " đơn vị" << std::endl;
    }

    //  Đọc UAV cho từng đơn vị
    std::cout << "4. Đọc UAV từ thư mục " << unitsFolder << "..." << std::endl;
    if (!unitList.loadUAVsFromPerUnitFiles(unitsFolder, uavPrefix, uavExt)) {
        std::cerr << "    Cảnh báo: Một số file UAV không đọc được!" << std::endl;
    }

    //  Đọc target
    std::cout << "5. Đọc target từ D:\\VS_Prj\\Graph\\x64\\Debug\\Data\\Data_target.csv..." << std::endl;
    if (!ReadTargetFile("D:\\VS_Prj\\Graph\\x64\\Debug\\Data\\Data_target.csv")) {
        std::cerr << "    Lỗi đọc file target!" << std::endl;
    }
    else {
        std::cout << "    Đã đọc " << targets_.size() << " target" << std::endl;
    }

    //  In tổng kết
    std::cout << "\n========== TỔNG KẾT ==========\n";
    std::cout << "Vertices: " << vertices_.size() << std::endl;
    std::cout << "Edges: " << edges_.size() << std::endl;
    std::cout << "Đơn vị: " << unitList.getUnitCount() << std::endl;
    std::cout << "Tổng số UAV: " << unitList.getTotalUAVCount() << std::endl;

    return success;
}
Vertex Graph::GetVertexById(int id) const {
    auto it = idIndexMap_.find(id);
    if (it == idIndexMap_.end()) {
        std::cerr << "[ERROR] Vertex id " << id << " not found!\n";
        return Vertex(); // trả về rỗng
    }
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



// Graph.cpp - thêm ở cuối file, trước phần Win32

void Graph::removeVertexZero()
{
    auto it = idIndexMap_.find(0);
    if (it != idIndexMap_.end()) {
        int index = it->second;
        vertices_.erase(vertices_.begin() + index);
        idIndexMap_.erase(it);

        // Cập nhật lại idIndexMap_ cho các vertex còn lại
        for (size_t i = 0; i < vertices_.size(); i++) {
            idIndexMap_[vertices_[i].id] = static_cast<int>(i);
        }
        std::cout << "Đã xóa vertex 0 khỏi đồ thị" << std::endl;
    }
}




// --- Win32 boilerplate unchanged below (kept from original) ---

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR    lpCmdLine,
    _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    // Initialize global strings
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_GRAPH, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    AllocConsole();
    FILE* fp;
    freopen_s(&fp, "CONOUT$", "w", stdout);
    freopen_s(&fp, "CONOUT$", "w", stderr);

    if (!InitInstance(hInstance, nCmdShow)) return FALSE;

    if (g_graph.readAllData(
        "D:\\VS_Prj\\Graph\\x64\\Debug\\Data\\UnitUAV.csv",
        "D:\\VS_Prj\\Graph\\x64\\Debug\\Data\\Vertex.csv",
        "D:\\VS_Prj\\Graph\\x64\\Debug\\Data\\Edge.csv",
        "D:\\VS_Prj\\Graph\\x64\\Debug\\Data\\units"
    )) {
        std::cout << "\n Đọc dữ liệu THÀNH CÔNG!" << std::endl;

        // Gán dữ liệu cho renderer
        g_renderer.setGraph(g_graph);
        g_renderer.setUnitList(g_graph.getUnitList());
        g_renderer.resetView();

        OptimizationProblem prob =
            OptimizationBuilder::build(g_graph.getUnitList(), g_graph);
        std::cout << "UAV count   = " << prob.uavs.size() << "\n";
        std::cout << "Target count= " << prob.targets.size() << "\n";
        const AssignmentSolution& best = prob.bestSolution;

       
        std::cout << "\n===== ASSIGNMENT RESULT =====\n";
        for (int i = 0; i < best.nUavTypes; i++) {
            for (int j = 0; j < best.nTargets; j++) {
                if (best.at(i, j) == 1) {
                    std::cout << "UAV type " << i
                        << " attacks Target " << j << "\n";
                }
            }
        }
        std::cout << "Fitness = " << best.fitness << "\n";
        std::cout << "=============================\n";

        g_renderer.setAssignment(best);
        InvalidateRect(hWnd, NULL, TRUE);

    }
    else
    {
        std::cerr << "\n LỖI đọc dữ liệu!" << std::endl;
    }



    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_GRAPH));
    MSG msg;

    while (GetMessage(&msg, nullptr, 0, 0))
    {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }

    return (int)msg.wParam;
}

ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;
    wcex.cbSize = sizeof(WNDCLASSEX);
    wcex.style = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc = WndProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = hInstance;
    wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_GRAPH));
    wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wcex.lpszMenuName = MAKEINTRESOURCEW(IDC_GRAPH);
    wcex.lpszClassName = szWindowClass;
    wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));
    return RegisterClassExW(&wcex);
}

BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
    hInst = hInstance;
    hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);
    if (!hWnd) return FALSE;
    ShowWindow(hWnd, nCmdShow);
    UpdateWindow(hWnd);
    return TRUE;
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hWnd, &ps);

        RECT clientRect;
        GetClientRect(hWnd, &clientRect);

        g_renderer.draw(hdc, clientRect);

        EndPaint(hWnd, &ps);
    }
    break;

    case WM_SIZE:
        InvalidateRect(hWnd, NULL, TRUE);
        break;

    case WM_MOUSEWHEEL:
    {
        int delta = GET_WHEEL_DELTA_WPARAM(wParam);
        if (delta > 0)
            g_renderer.zoomIn();
        else
            g_renderer.zoomOut();
        InvalidateRect(hWnd, NULL, TRUE);
    }
    break;

    case WM_LBUTTONDOWN:
        SetCapture(hWnd);
        break;

    case WM_LBUTTONUP:
        ReleaseCapture();
        break;

    case WM_MOUSEMOVE:
    {
        static int lastX = -1;
        static int lastY = -1;

        if (wParam & MK_LBUTTON)
        {
            int x = LOWORD(lParam);
            int y = HIWORD(lParam);

            if (lastX != -1 && lastY != -1)
            {
                int dx = x - lastX;
                int dy = y - lastY;
                g_renderer.pan(dx, dy);
                InvalidateRect(hWnd, NULL, TRUE);
            }

            lastX = x;
            lastY = y;
        }
        else
        {
            lastX = lastY = -1;
        }
    }
    break;


    case WM_COMMAND:
    {
        int wmId = LOWORD(wParam);
        switch (wmId)
        {
        case IDM_ABOUT:
            DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
            break;
        case IDM_EXIT:
            DestroyWindow(hWnd);
            break;
        default:
            return DefWindowProc(hWnd, message, wParam, lParam);
        }
    }
    break;

    case WM_DESTROY:
        PostQuitMessage(0);
        break;

    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG: return (INT_PTR)TRUE;
    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}