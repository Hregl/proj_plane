// ============================================================================
//  src/infrastructure/config/TurntablePermitFile.cpp
//
//  见头文件说明。解析规则与 `tools/turntable_acceptance.cpp` 原先的
//  本地实现逐条一致；两处现已共用本文件，不再各写一份。
// ============================================================================

#include "infrastructure/config/TurntablePermitFile.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <vector>

namespace aircraft
{
namespace infrastructure
{

namespace
{

std::string trim(const std::string& s)
{
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
    {
        return std::string();
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

bool parseLongLong(const std::string& text, long long& out)
{
    if (text.empty())
    {
        return false;
    }
    char* end = nullptr;
    errno = 0;
    const long long value = std::strtoll(text.c_str(), &end, 10);
    if (errno != 0 || end == nullptr || *end != '\0')
    {
        return false;
    }
    out = value;
    return true;
}

bool parseDouble(const std::string& text, double& out)
{
    if (text.empty())
    {
        return false;
    }
    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(text.c_str(), &end);
    if (errno != 0 || end == nullptr || *end != '\0' ||
        !std::isfinite(value))
    {
        return false;
    }
    out = value;
    return true;
}

}  // namespace

bool loadTurntablePermitFile(const std::string& path,
                             data::TurntableAcceptancePermit& permit,
                             std::string& error)
{
    std::ifstream input(path);
    if (!input)
    {
        error = "无法打开许可文件：" + path;
        return false;
    }

    std::vector<std::string> seen;
    auto markSeen = [&](const std::string& key) {
        for (const auto& s : seen)
        {
            if (s == key)
            {
                return false;
            }
        }
        seen.push_back(key);
        return true;
    };

    std::string line;
    int lineNo = 0;
    while (std::getline(input, line))
    {
        ++lineNo;
        const std::string stripped = trim(line);
        if (stripped.empty() || stripped[0] == '#')
        {
            continue;
        }
        const auto eq = stripped.find('=');
        if (eq == std::string::npos)
        {
            error = "许可第 " + std::to_string(lineNo) + " 行缺少 '='";
            return false;
        }
        const std::string key = trim(stripped.substr(0, eq));
        std::string value = trim(stripped.substr(eq + 1));
        if (!value.empty() && value.front() == '"' && value.back() == '"')
        {
            value = value.substr(1, value.size() - 2);
        }
        if (!markSeen(key))
        {
            error = "许可键重复：" + key;
            return false;
        }

        long long i = 0;
        double d = 0.0;
        if (key == "scope_id")
        {
            permit.scopeId = value;
        }
        else if (key == "device_host")
        {
            permit.deviceHost = value;
        }
        else if (key == "device_port")
        {
            if (!parseLongLong(value, i))
            {
                error = "device_port 不是整数";
                return false;
            }
            permit.devicePort = static_cast<int>(i);
        }
        else if (key == "pelco_address")
        {
            if (!parseLongLong(value, i))
            {
                error = "pelco_address 不是整数";
                return false;
            }
            permit.pelcoAddress = static_cast<int>(i);
        }
        else if (key == "allow_absolute_position")
        {
            if (!parseLongLong(value, i) || (i != 0 && i != 1))
            {
                error = "allow_absolute_position 必须是 0 或 1";
                return false;
            }
            permit.allowAbsolutePosition = i == 1;
        }
        else if (key == "allow_stop_at_rest")
        {
            if (!parseLongLong(value, i) || (i != 0 && i != 1))
            {
                error = "allow_stop_at_rest 必须是 0 或 1";
                return false;
            }
            permit.allowStopFrameAtRest = i == 1;
        }
        else if (key == "azimuth_min_deg")
        {
            if (!parseDouble(value, d))
            {
                error = key + " 不是有限数";
                return false;
            }
            permit.azimuthMinDeg = d;
        }
        else if (key == "azimuth_max_deg")
        {
            if (!parseDouble(value, d))
            {
                error = key + " 不是有限数";
                return false;
            }
            permit.azimuthMaxDeg = d;
        }
        else if (key == "elevation_min_deg")
        {
            if (!parseDouble(value, d))
            {
                error = key + " 不是有限数";
                return false;
            }
            permit.elevationMinDeg = d;
        }
        else if (key == "elevation_max_deg")
        {
            if (!parseDouble(value, d))
            {
                error = key + " 不是有限数";
                return false;
            }
            permit.elevationMaxDeg = d;
        }
        else if (key == "max_displacement_deg")
        {
            if (!parseDouble(value, d))
            {
                error = key + " 不是有限数";
                return false;
            }
            permit.maxDisplacementDeg = d;
        }
        else if (key == "max_executions")
        {
            if (!parseLongLong(value, i))
            {
                error = "max_executions 不是整数";
                return false;
            }
            permit.maxExecutions = static_cast<int>(i);
        }
        else if (key == "valid_from_epoch_ms")
        {
            if (!parseLongLong(value, i))
            {
                error = "valid_from_epoch_ms 不是整数毫秒";
                return false;
            }
            permit.validFromEpochMs = i;
        }
        else if (key == "valid_until_epoch_ms")
        {
            if (!parseLongLong(value, i))
            {
                error = "valid_until_epoch_ms 不是整数毫秒";
                return false;
            }
            permit.validUntilEpochMs = i;
        }
        else
        {
            error = "许可含未知键：" + key;
            return false;
        }
    }

    const std::vector<std::string> required = {
        "scope_id", "device_host", "device_port", "pelco_address",
        "allow_absolute_position", "allow_stop_at_rest",
        "azimuth_min_deg", "azimuth_max_deg",
        "elevation_min_deg", "elevation_max_deg", "max_displacement_deg",
        "max_executions", "valid_from_epoch_ms", "valid_until_epoch_ms"};
    for (const auto& key : required)
    {
        bool present = false;
        for (const auto& s : seen)
        {
            if (s == key)
            {
                present = true;
                break;
            }
        }
        if (!present)
        {
            error = "许可缺少必需键：" + key;
            return false;
        }
    }
    return true;
}

}  // namespace infrastructure
}  // namespace aircraft
