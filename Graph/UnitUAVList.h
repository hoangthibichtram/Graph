#pragma once

#include "UnitUAV.h"

#include <string>
#include <vector>
#include <unordered_map>

class UnitUAVList
{
public:
    UnitUAVList() noexcept = default;

    bool loadUnitsFromFile(const std::string& path);
    bool loadUAVsFromCombinedFile(const std::string& path);
    bool loadUAVsFromPerUnitFiles(const std::string& folder,
        const std::string& prefix = "UAV_",
        const std::string& ext = ".csv");

    int getTotalUAVCount() const;

    UnitUAV* getUnitById(const std::string& unit_id) noexcept;
    const UnitUAV* getUnitById(const std::string& unit_id) const noexcept;

    const std::vector<UnitUAV>& getUnits() const noexcept { return units_; }
    int getUnitCount() const noexcept { return (int)units_.size(); }

private:
    std::vector<UnitUAV> units_;
    std::unordered_map<std::string, size_t> unit_index_map_;
    std::vector<UnitUAV>& getUnitsRef() { return units_; }

};
