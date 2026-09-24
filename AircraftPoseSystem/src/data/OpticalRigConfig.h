#pragma once

// ============================================================================
//  src/data/OpticalRigConfig.h
//
//  依据：ENG-09 §4.1、§6.2（类型定义冻结）、裁决 C-18
//        ENG-01 §3.2（配置文件清单）
//
//  作用：指向光学平台标定数据的**位置**（不是标定数据本身）。
//
//  ⚠ 为什么是"位置 + 版本号"而不是直接内联标定值：
//  标定数据体积大（三个焦段的内参矩阵、畸变系数、外参、rigToShip），
//  且由独立工具（SYS-11）产出为文件。配置只记录"去哪儿取、取哪一版"，
//  OpticalRigCalibration 才是数据本体。
//
//  ⚠ calibrationId 的作用（对应 ENG-09 §6.8 的配置版本追踪）：
//  测量结果包 measurement_xxx/ 必须能追溯到**当时用的哪一版标定**
//  （SYS-12 §17 / SYS-15 §16 要求保存 calibration_id）。
//  没有它就无法回答"这个 1.2 角分的偏差是算法的还是标定的"——
//  标定重做后旧结果包的解释力完全依赖这个字段。
//  因此它必须是**不可变标识**（如日期批次或哈希前缀），
//  不得用"latest"这类会随文件系统变化的写法。
// ============================================================================

#include <string>

namespace aircraft
{
namespace data
{

/// 光学平台标定配置（ENG-09 §6.2）。
struct OpticalRigConfig
{
    /// 标定文件所在目录。对应 ENG-01 §3.3 的标定数据目录。
    std::string calibrationDir;

    /// 标定版本标识，写入结果包以供追溯（不得使用 "latest"）。
    std::string calibrationId;
};

}  // namespace data
}  // namespace aircraft
