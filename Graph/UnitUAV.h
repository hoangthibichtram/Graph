#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <iostream>

// =========================
// CLASS UAV (duy nhất)
// =========================
class UAV
{
private:
    int uav_id;
    std::string uav_code;
    std::string uav_type;
    float range;
    float speed;
    std::string weapon;
    float explosize;
    float radius;
    double cost_usd;
    std::string unit_id;

public:
    UAV()
        : uav_id(-1), range(0), speed(0),
        explosize(0), radius(0), cost_usd(0)
    {
    }

    UAV(int id, const std::string& code, const std::string& type,
        float rng, float spd, const std::string& wpn,
        float expl, float rad, double cost, const std::string& unit)
        : uav_id(id), uav_code(code), uav_type(type),
        range(rng), speed(spd), weapon(wpn),
        explosize(expl), radius(rad), cost_usd(cost),
        unit_id(unit)
    {
    }

    // Getter
    int getId() const { return uav_id; }
    const std::string& getCode() const { return uav_code; }
    const std::string& getType() const { return uav_type; }
    float getRange() const { return range; }
    float getSpeed() const { return speed; }
    const std::string& getWeapon() const { return weapon; }
    float getExplosize() const { return explosize; }
    float getRadius() const { return radius; }
    double getCostUsd() const { return cost_usd; }
    const std::string& getUnitId() const { return unit_id; }

    // Setter
    void setId(int id) { uav_id = id; }
    void setCode(const std::string& code) { uav_code = code; }
    void setType(const std::string& type) { uav_type = type; }
    void setRange(float rng) { range = rng; }
    void setSpeed(float spd) { speed = spd; }
    void setWeapon(const std::string& wpn) { weapon = wpn; }
    void setExplosize(float expl) { explosize = expl; }
    void setRadius(float rad) { radius = rad; }
    void setCostUsd(double cost) { cost_usd = cost; }
    void setUnitId(const std::string& unit) { unit_id = unit; }

    void printInfo() const
    {
        std::cout << "  UAV " << uav_code << " (" << uav_type << ")"
            << " | Vũ khí: " << weapon
            << " | Lượng nổ: " << explosize << "kg"
            << " | Tầm: " << range << "km"
            << " | Tốc độ: " << speed << "km/h"
            << " | Chi phí: $" << cost_usd
            << " | Đơn vị: " << unit_id << std::endl;
    }
};


// =========================
// CLASS UnitUAV
// =========================
class UnitUAV
{
public:
    UnitUAV() noexcept;
    UnitUAV(std::string unitId, std::string unitName,
        double x = 0.0, double y = 0.0, double z = 0.0) noexcept;

    bool readUnitInfo(const std::string& path);
    bool readUAVsFromFile(const std::string& path);

    void addUAV(const UAV& uav);

    UAV* getUAVById(int uav_id) noexcept;
    const UAV* getUAVById(int uav_id) const noexcept;

    std::vector<UAV> getUAVsByType(const std::string& type) const;

    int getUAVCount() const noexcept;

    double getX() const noexcept { return base_x; }
    double getY() const noexcept { return base_y; }
    double getZ() const noexcept { return base_z; }

    const std::string& getUnitId() const noexcept { return unit_id; }
    const std::string& getUnitName() const noexcept { return unit_name; }

    const std::vector<UAV>& getUAVs() const noexcept { return uavs; }

private:
    std::string unit_id;
    std::string unit_name;
    double base_x;
    double base_y;
    double base_z;

    std::vector<UAV> uavs;
    std::unordered_map<int, int> uav_index_map; // uav_id -> index
};
