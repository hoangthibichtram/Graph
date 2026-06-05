#include "UnitUAVList.h"
#include "OptimizationProblem.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>

// Local helpers (consistent with other CSV readers)
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

UnitUAV* UnitUAVList::getUnitById(const std::string& unit_id) noexcept
{
    auto it = unit_index_map_.find(unit_id);
    if (it == unit_index_map_.end()) return nullptr;
    return &units_[it->second];
}

const UnitUAV* UnitUAVList::getUnitById(const std::string& unit_id) const noexcept
{
    auto it = unit_index_map_.find(unit_id);
    if (it == unit_index_map_.end()) return nullptr;
    return &units_[it->second];
}


// Đọc danh sách đơn vị từ file CSV (UnitUAV.csv)
// File header được tìm theo tên cột linh hoạt: unit_id, unit_name, x, y, z
bool UnitUAVList::loadUnitsFromFile(const std::string& path)
{
    std::cout << "loadUnitsFromFile: " << path << std::endl;

    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "  KHÔNG THỂ MỞ FILE!" << std::endl;
        return false;
    }
    std::cout << "  MỞ FILE THÀNH CÔNG!" << std::endl;

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

    int idxUnitId = get({ "unit_id","unitid","id" });
    int idxUnitName = get({ "unit_name","name","unitname" });
    int idxX = get({ "x","lon","longitude" });
    int idxY = get({ "y","lat","latitude" });
    int idxZ = get({ "z","alt","altitude" });

    std::string line;
    int added = 0;
    while (std::getline(ifs, line))
    {
        RemoveUTF8BOM(line);
        line = Trim(line);
        if (line.empty()) continue;
        if (line[0] == '#' || line[0] == '/') continue;

        auto tok = ParseCsvLine(line, delim);
        try
        {
            std::string u_id;
            std::string u_name;
            double x = 0.0, y = 0.0, z = 0.0;

            if (idxUnitId >= 0 && idxUnitId < static_cast<int>(tok.size())) u_id = tok[idxUnitId];
            if (idxUnitName >= 0 && idxUnitName < static_cast<int>(tok.size())) u_name = tok[idxUnitName];
            if (idxX >= 0 && idxX < static_cast<int>(tok.size())) x = std::stod(tok[idxX]);
            if (idxY >= 0 && idxY < static_cast<int>(tok.size())) y = std::stod(tok[idxY]);
            if (idxZ >= 0 && idxZ < static_cast<int>(tok.size())) z = std::stod(tok[idxZ]);

            if (u_id.empty()) continue;

            auto it = unit_index_map_.find(u_id);
            if (it == unit_index_map_.end())
            {
                UnitUAV u(u_id, u_name, x, y, z);

                std::cout << "[UNIT] " << u_id
                    << " name=" << u_name
                    << " x=" << x
                    << " y=" << y
                    << " z=" << z << "\n";

                units_.push_back(std::move(u));
                unit_index_map_[u_id] = units_.size() - 1;
                ++added;
            }
            else
            {
                // Nếu đơn vị đã tồn tại, cập nhật tên/tọa độ nếu rỗng/zero.
                UnitUAV& existing = units_[it->second];
                // không có setter công khai trong hiện tại; nếu cần cập nhật, thêm setter vào UnitUAV.
                (void)existing;
            }
        }
        catch (...) { continue; }
    }

    return added > 0;
}

// Đọc các file UAV riêng cho từng đơn vị trong thư mục.
// Mỗi file có tên: prefix + unit_id + ext
// Ví dụ: folder="data", prefix="UAV_", ext=".csv" -> "data/UAV_SQ1.csv"
bool UnitUAVList::loadUAVsFromPerUnitFiles(const std::string& folder,
    const std::string& prefix,
    const std::string& ext)
{
    bool anyAdded = false;

    for (auto& unit : units_)
    {
        std::string filename = folder;
        if (!filename.empty() && filename.back() != '/' && filename.back() != '\\')
            filename.push_back('/');
        filename += "data_uav_" + unit.getUnitId() + ext;

        if (unit.readUAVsFromFile(filename))
        {
            // ⭐ CHUYỂN UAV từ UnitUAV → UAVTypeOpt
            for (const auto& u : unit.getUAVs())
            {
                UAVTypeOpt opt;
                opt.id = u.getId();
                opt.code = u.getCode();
                opt.costPerAttack = u.getCostUsd();
                opt.maxCount = 1;
                opt.maxBudget = u.getCostUsd();
                opt.unitIndex = unit_index_map_[unit.getUnitId()];
                opt.unitName = unit.getUnitName();

                // Khởi tạo vector aij/pij theo số mục tiêu (tạm để trống, OptimizationBuilder sẽ fill)
            }

            anyAdded = true;
        }
    }

    return anyAdded;
}


bool UnitUAVList::loadUAVsFromCombinedFile(const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cout << "[UAV] LOI: Khong mo duoc file: " << path << "\n";
        return false;
    }

    std::string header;
    if (!std::getline(ifs, header)) return false;
    RemoveUTF8BOM(header);
    header = Trim(header);
    char delim = DetectDelimiter(header);
    auto hdr = ParseCsvLine(header, delim);

    // Map tên cột → index (không phân biệt hoa thường)
    std::unordered_map<std::string, size_t> hidx;
    for (size_t i = 0; i < hdr.size(); ++i)
        hidx[ToLower(hdr[i])] = i;

    auto col = [&](std::initializer_list<const char*> names) -> int {
        for (auto n : names) {
            auto it = hidx.find(ToLower(n));
            if (it != hidx.end()) return (int)it->second;
        }
        return -1;
        };

    // Tìm index từng cột — chấp nhận cả "explosize" lẫn "explosive"
    int iId = col({ "uav_id","id" });
    int iCode = col({ "uav_code","code" });
    int iType = col({ "uav_type","type" });
    int iRange = col({ "range" });
    int iSpeed = col({ "speed" });
    int iWeapon = col({ "weapon" });
    int iExp = col({ "explosive","explosize","expl" });
    int iRadius = col({ "radius" });
    int iCost = col({ "cost_usd","cost","price" });
    int iUnit = col({ "unit_id","unit","unitid" });

    std::string line;
    int added = 0;
    while (std::getline(ifs, line))
    {
        RemoveUTF8BOM(line);
        line = Trim(line);
        if (line.empty() || line[0] == '#' || line[0] == '/') continue;

        auto tok = ParseCsvLine(line, delim);
        try
        {
            // Lấy unit_id để tìm đúng tiểu đội
            std::string unitId;
            if (iUnit >= 0 && iUnit < (int)tok.size())
                unitId = tok[iUnit];
            if (unitId.empty()) continue;

            // Tìm UnitUAV tương ứng
            UnitUAV* unit = getUnitById(unitId);
            if (!unit) {
                std::cout << "[UAV] CANH BAO: Khong tim thay don vi '"
                    << unitId << "'\n";
                continue;
            }

            // Đọc từng trường
            UAV u;
            if (iId >= 0) u.setId(std::stoi(tok[iId]));
            if (iCode >= 0) u.setCode(tok[iCode]);
            if (iType >= 0) u.setType(tok[iType]);
            if (iRange >= 0) u.setRange(std::stof(tok[iRange]));
            if (iSpeed >= 0) u.setSpeed(std::stof(tok[iSpeed]));
            if (iWeapon >= 0) u.setWeapon(tok[iWeapon]);
            if (iExp >= 0) u.setExplosize(std::stof(tok[iExp]));
            if (iRadius >= 0) u.setRadius(std::stof(tok[iRadius]));
            if (iCost >= 0) u.setCostUsd(std::stod(tok[iCost]));
            u.setUnitId(unitId);

            unit->addUAV(u);
            added++;

            std::cout << "[UAV] " << u.getCode()
                << " -> don vi " << unitId
                << " | explosive=" << u.getExplosive()
                << " | cost=" << u.getCostUsd() << "\n";
        }
        catch (...) {
            std::cout << "[UAV] LOI parse dong: " << line << "\n";
        }
    }

    std::cout << "[UAV] Da tai " << added << " UAV tu " << path << "\n";
    return added > 0;
}
int UnitUAVList::getTotalUAVCount() const
{
    int total = 0;
    for (const auto& unit : units_)
    {
        total += unit.getUAVCount();
    }
    return total;
}
const UnitUAV& UnitUAVList::getUnit(size_t index) const
{
    return units_.at(index);
}
int UnitUAVList::getUnitIndex(const std::string& unitId) const
{
    const auto& units = getUnits();
    for (int i = 0; i < units.size(); i++)
    {
        if (units[i].getUnitId() == unitId)
            return i;
    }
    return -1; // không tìm thấy
}




