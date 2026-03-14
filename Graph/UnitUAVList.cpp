#include "UnitUAVList.h"

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
        header = Trim(line);

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
bool UnitUAVList::loadUAVsFromPerUnitFiles(const std::string& folder, const std::string& prefix, const std::string& ext)
{
    bool anyAdded = false;
    for (auto& unit : units_)
    {
        // build filename
        std::string filename = folder;
        if (!filename.empty() && filename.back() != '/' && filename.back() != '\\') filename.push_back('/');
        filename += "data_uav_" + unit.getUnitId() + ext;


        // delegate đọc file cho UnitUAV (UnitUAV::readUAVsFromFile sẽ kiểm tra unit_id nếu cần)
        if (unit.readUAVsFromFile(filename))
        {
            anyAdded = true;
        }
    }
    return anyAdded;
}

// (không bắt buộc) giữ nguyên để khả năng đọc file tổng hợp nếu cần trong tương lai
bool UnitUAVList::loadUAVsFromCombinedFile(const std::string& path)
{
    // Giữ interface nhưng trả false nhanh để phản ánh yêu cầu hiện tại:
    // Người dùng muốn dùng file riêng cho từng đơn vị, không cần phân phối từ file tổng hợp.
    // Nếu bạn vẫn muốn hỗ trợ file tổng hợp, hãy yêu cầu để bật lại.
    (void)path;
    return false;
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




