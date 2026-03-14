#include "UnitUAV.h"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <unordered_map>

// Helpers
namespace
{
    static inline std::string Trim(const std::string& s)
    {
        const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
        auto begin = std::find_if(s.begin(), s.end(),
            [](unsigned char ch) { return !std::isspace(ch); });

        auto end = std::find_if(s.rbegin(), s.rend(),
            [](unsigned char ch) { return !std::isspace(ch); }).base();

        if (begin >= end) return {};
        return std::string(begin, end);
    }

    static inline std::string ToLower(std::string s)
    {
        for (auto& ch : s) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
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

    static inline char DetectDelimiter(const std::string& sample)
    {
        size_t c = std::count(sample.begin(), sample.end(), ',');
        size_t s = std::count(sample.begin(), sample.end(), ';');
        return (s > c) ? ';' : ',';
    }
}

// ===== UnitUAV implementation =====

UnitUAV::UnitUAV() noexcept
    : unit_id(), unit_name(), base_x(0.0), base_y(0.0), base_z(0.0)
{
}

UnitUAV::UnitUAV(std::string unitId, std::string unitName,
    double x, double y, double z) noexcept
    : unit_id(std::move(unitId)), unit_name(std::move(unitName)),
    base_x(x), base_y(y), base_z(z)
{
}

bool UnitUAV::readUnitInfo(const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    std::string header;
    while (std::getline(ifs, header))
    {
        header = Trim(header);
        if (header.empty()) continue;
        if (header[0] == '#' || header[0] == '/') continue;
        break;
    }
    if (header.empty()) return false;

    char delim = DetectDelimiter(header);
    auto hdr = ParseCsvLine(header, delim);
    std::unordered_map<std::string, size_t> hidx;
    for (size_t i = 0; i < hdr.size(); ++i) hidx[ToLower(hdr[i])] = i;

    int idxUnitId = hidx.count("unit_id") ? (int)hidx["unit_id"] : -1;
    int idxUnitName = hidx.count("unit_name") ? (int)hidx["unit_name"] : -1;
    int idxX = hidx.count("x") ? (int)hidx["x"] : -1;
    int idxY = hidx.count("y") ? (int)hidx["y"] : -1;
    int idxZ = hidx.count("z") ? (int)hidx["z"] : -1;

    std::string line;
    while (std::getline(ifs, line))
    {
        line = Trim(line);
        if (line.empty()) continue;
        if (line[0] == '#' || line[0] == '/') continue;

        auto tok = ParseCsvLine(line, delim);
        try
        {
            if (idxUnitId >= 0 && idxUnitId < (int)tok.size()) unit_id = tok[idxUnitId];
            if (idxUnitName >= 0 && idxUnitName < (int)tok.size()) unit_name = tok[idxUnitName];
            if (idxX >= 0 && idxX < (int)tok.size()) base_x = std::stod(tok[idxX]);
            if (idxY >= 0 && idxY < (int)tok.size()) base_y = std::stod(tok[idxY]);
            if (idxZ >= 0 && idxZ < (int)tok.size()) base_z = std::stod(tok[idxZ]);
            return true;
        }
        catch (...) { continue; }
    }

    return false;
}

bool UnitUAV::readUAVsFromFile(const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    std::string header;
    if (!std::getline(ifs, header)) return false;
    RemoveUTF8BOM(header);
    char delim = DetectDelimiter(header);
    auto hdr = ParseCsvLine(header, delim);
    std::unordered_map<std::string, size_t> hidx;
    for (size_t i = 0; i < hdr.size(); ++i) hidx[ToLower(hdr[i])] = i;

    auto get = [&](std::initializer_list<const char*> names)->int {
        for (auto n : names) {
            auto it = hidx.find(ToLower(n));
            if (it != hidx.end()) return (int)it->second;
        }
        return -1;
        };

    int idxUavId = get({ "uav_id","id","uavid" });
    int idxCode = get({ "uav_code","code" });
    int idxType = get({ "uav_type","type" });
    int idxRange = get({ "range" });
    int idxSpeed = get({ "speed" });
    int idxWeapon = get({ "weapon" });
    int idxExpl = get({ "explosize","explosive","expl" });
    int idxRadius = get({ "radius" });
    int idxCost = get({ "cost_usd","cost","price" });
    int idxUnitId = get({ "unit_id","unit","unitid" });

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
            UAV u;

            if (idxUavId >= 0 && idxUavId < (int)tok.size()) u.setId(std::stoi(tok[idxUavId]));
            if (idxCode >= 0 && idxCode < (int)tok.size()) u.setCode(tok[idxCode]);
            if (idxType >= 0 && idxType < (int)tok.size()) u.setType(tok[idxType]);
            if (idxRange >= 0 && idxRange < (int)tok.size()) u.setRange(std::stof(tok[idxRange]));
            if (idxSpeed >= 0 && idxSpeed < (int)tok.size()) u.setSpeed(std::stof(tok[idxSpeed]));
            if (idxWeapon >= 0 && idxWeapon < (int)tok.size()) u.setWeapon(tok[idxWeapon]);
            if (idxExpl >= 0 && idxExpl < (int)tok.size()) u.setExplosize(std::stof(tok[idxExpl]));
            if (idxRadius >= 0 && idxRadius < (int)tok.size()) u.setRadius(std::stof(tok[idxRadius]));
            if (idxCost >= 0 && idxCost < (int)tok.size()) u.setCostUsd(std::stod(tok[idxCost]));
            if (idxUnitId >= 0 && idxUnitId < (int)tok.size()) u.setUnitId(tok[idxUnitId]);

            // Nếu file có cột unit_id thì chỉ nhận UAV đúng đơn vị
            if (!u.getUnitId().empty() && !unit_id.empty())
            {
                if (u.getUnitId() != unit_id) continue;
            }

            addUAV(u);
        }
        catch (...) { continue; }
    }

    return true;
}

void UnitUAV::addUAV(const UAV& uav)
{
    int idx = (int)uavs.size();
    uavs.push_back(uav);
    uav_index_map[uav.getId()] = idx;
}

UAV* UnitUAV::getUAVById(int uav_id) noexcept
{
    auto it = uav_index_map.find(uav_id);
    if (it == uav_index_map.end()) return nullptr;
    int idx = it->second;
    if (idx < 0 || (size_t)idx >= uavs.size()) return nullptr;
    return &uavs[idx];
}

const UAV* UnitUAV::getUAVById(int uav_id) const noexcept
{
    auto it = uav_index_map.find(uav_id);
    if (it == uav_index_map.end()) return nullptr;
    int idx = it->second;
    if (idx < 0 || (size_t)idx >= uavs.size()) return nullptr;
    return &uavs[idx];
}

std::vector<UAV> UnitUAV::getUAVsByType(const std::string& type) const
{
    std::vector<UAV> out;
    for (const auto& u : uavs)
    {
        if (u.getType() == type) out.push_back(u);
    }
    return out;
}

int UnitUAV::getUAVCount() const noexcept
{
    return (int)uavs.size();
}
