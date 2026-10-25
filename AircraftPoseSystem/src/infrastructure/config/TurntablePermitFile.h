#pragma once

// ============================================================================
//  src/infrastructure/config/TurntablePermitFile.h
//
//  作用：读取**本次运动许可**文件（显式路径，不来自 config/）。
//
//  ⚠ 这是与 `tools/turntable_acceptance.cpp` 共用的**唯一**解析器：
//    主程序与验收入口必须按同一套键名、同一套重复键/未知键/缺键判据
//    读取许可。两处各写一份解析器时，一条规则只改了一边，另一边仍会
//    接受（或拒绝）同一份文件 —— 现场看到的是“同一个许可，两个结论”。
//
//  ⚠ 本函数只做**文件结构与取值类型**校验，不做设备/范围/期限/次数
//    的业务校验。后者由生产会话在 `enableAcceptancePermit()` 中完成：
//    许可里的 host/port/address 必须与配置一致、范围必须在机械行程内、
//    有效期必须有效且未过期、次数必须 > 0。把业务校验放在解析器里会让
//    “文件读对了”和“许可被接受”变成同一句话，无法分别排查。
// ============================================================================

#include <string>

#include "data/TurntableAcceptancePermit.h"

namespace aircraft
{
namespace infrastructure
{

/// 读取许可文件。
///
/// @param path   许可文件路径（显式传入，不来自配置目录）。
/// @param permit 输出；成功时填满全部必需键。
/// @param error  失败原因（人读）。
/// @return true = 文件结构合法且所有必需键都存在；false = 失败。
///
/// 判据：
///   · 空行与 `#` 开头的行忽略；
///   · 每行必须是 `key = value`；
///   · 键不得重复；未知键即失败（避免拼写错误被静默忽略）；
///   · 以下键为必需：scope_id、device_host、device_port、pelco_address、
///     allow_absolute_position、allow_stop_at_rest、azimuth_min_deg、
///     azimuth_max_deg、elevation_min_deg、elevation_max_deg、
///     max_displacement_deg、max_executions、valid_from_epoch_ms、
///     valid_until_epoch_ms；
///   · 数值必须能完整解析（不接受 "1.0x"、非有限数）。
bool loadTurntablePermitFile(const std::string& path,
                             data::TurntableAcceptancePermit& permit,
                             std::string& error);

}  // namespace infrastructure
}  // namespace aircraft
