#pragma once

// ============================================================================
//  src/data/RawImagePayload.h
//
//  依据：ENG-09 V2.3 §5.28（原始载荷契约，冻结）、§6.5（结果包元数据）
//        SYS-04 V2.4 §4.1 / SYS-05 V2.2 §5.1（ImageFrame）
//        裁决 C-01 v1.7
//
//  作用：把**相机交付的原始字节**连同解释它所需的全部信息一起携带，
//  使"保存原始数据"这件事不依赖任何运行期上下文（配置、文件名、
//  逻辑通道号）。理由见下。
//
//  ⚠ 为什么必须**真正拥有字节**，而不是持有 SDK 缓冲区的指针：
//  `IMV_GetFrame` 交出的是 **SDK 内部缓存**（§2.5 已核：必须
//  `IMV_ReleaseFrame`，样例里"取到就立刻在同一条执行线上释放"）。
//  若 ImageFrame 只是浅引用该缓冲，则释放之后（甚至下一帧取到同一块
//  复用的缓冲之后）已交付的帧内容会被**就地改写**，而观感是"保存下来的
//  raw 偶尔对不上",排查方向会指向磁盘 I/O 或文件系统。
//  故本结构体的契约是：**归还 SDK 缓冲区之前完成复制**，
//  且**发布后不再修改**（`shared_ptr<const vector>` 把这条写进类型）。
//
//  ⚠ 为什么**每帧一份独立缓冲**、不做缓冲池复用：池复用会把 SDK 侧
//  的缓冲复用问题搬进后端内部 —— 调用方（预览、测量缓存、Recorder）
//  可能同时持有若干帧的引用，池一旦复用其中一块，被复用的那帧就会
//  在持有者毫无察觉的情况下改变内容。一份一缓冲的代价是每帧一次分配，
//  与"取一帧就要 memcpy 一次 SDK 缓冲"相比不增加数量级。
//  ⚠ 本文件与 ImageFrame.h 的注释里**不得**写"shared_ptr 使浅拷贝天然
//  安全"：安全来自**自有 + 生命周期正确 + 发布后不变**三件事，
//  与智能指针本身无关 —— 若字节是别处的（shared_ptr 也是这样），
//  浅拷贝照样不安全。
// ============================================================================

#include <cstdint>
#include <memory>
#include <vector>

#include "data/PixelFormat.h"

namespace aircraft
{
namespace data
{

/// 单个像素在**本批声明的紧凑契约**下的字节数（用分子/分母表达，
/// 因为 `Mono12Packed` 是 1.5 字节/像素）。
///
/// 契约内容：载荷按"每像素 `numerator/denominator` 字节、**无行尾填充、
/// 无整帧填充**"的紧凑布局解释。这是**本批明确声明的契约**（可与配置里的
/// 格式断言一一对应），**不是**从字节数"推"出来的结论，也**不是**
/// SDK 的任何保证（SDK 的 `paddingX/paddingY` 语义未文档化，§2.5/§2.6）。
struct CompactLayout
{
    uint32_t bytesNumerator   = 0;      ///< 分子（每像素字节数 × denominator）
    uint32_t bytesDenominator = 1;      ///< 分母
    bool     supported        = false;  ///< 本批是否实现该格式
};

/// 本批支持的三种格式的紧凑布局；`Mono12Packed` 返回 `supported = false`。
constexpr CompactLayout compactLayoutOf(PixelFormat format)
{
    switch (format)
    {
    case PixelFormat::Mono8:
        return CompactLayout{1, 1, true};
    case PixelFormat::Mono12:
        // SDK 的 OCCUPY16BIT：12 位有效值占 2 字节容器。
        return CompactLayout{2, 1, true};
    case PixelFormat::BGR8:
        return CompactLayout{3, 1, true};
    case PixelFormat::Mono12Packed:
        return CompactLayout{3, 2, false};
    }
    return CompactLayout{0, 1, false};
}

/// 该格式的有效位对齐（本批只产出 `LsbZeroPadded`）。
///
/// ⚠ 参数**当前不参与判定**，且这是有意的而非遗漏：本批支持的三种格式
/// （`Mono8`／`Mono12`／`BGR8`）的有效位位置只有一种约定 —— PFNC 2.4
/// §6.1.1 的低位对齐、高位补零（16 位容器的 `Mono12` 是唯一非整字节者）；
/// `Mono8`／`BGR8` 是整字节格式，"对齐"概念不适用，返回同一取值是为了让
/// 字段总有一个确定取值，而不是留一个"对 8 位无意义"的第二语义
/// （那会让 Recorder 的元数据出现两种解读）。
/// 保留参数是为了签名与同族函数（`validBitsOf` 等）一致，且将来
/// `Mono12Packed` 真正实现时其对齐约定**可能不同**，加分支不必改调用点。
///
/// `BitAlignment::MsbAligned` / `Unknown` 在本批**不可达**：出现即契约错误
/// （由 `isValidCombination` 拒绝），**不是**"未知就失败"的运行期分支。
constexpr BitAlignment bitAlignmentOf(PixelFormat format)
{
    (void)format;
    return BitAlignment::LsbZeroPadded;
}

/// 该格式的有效位数。
constexpr uint16_t validBitsOf(PixelFormat format)
{
    switch (format)
    {
    case PixelFormat::Mono8:
        return 8;
    case PixelFormat::BGR8:
        return 8;
    case PixelFormat::Mono12:
        return 12;
    case PixelFormat::Mono12Packed:
        return 12;
    }
    return 0;
}

/// 该格式在**本批**要求的打包方式。
constexpr Packing requiredPackingOf(PixelFormat format)
{
    return format == PixelFormat::Mono12Packed ? Packing::Packed12
                                               : Packing::Unpacked;
}

/// 合法组合判定（ENG-09 V2.3 §5.28 第 1 条的组合表）。
///
/// 单一实现，由后端发布帧时断言、Recorder 落盘时复检 ——
/// 两处各写一份必然会在某次修改后分叉，而分叉的表现是
/// "后端放行的帧被 Recorder 拒绝"，看起来像 I/O 故障。
constexpr bool isValidCombination(PixelFormat format,
                                  Packing      packing,
                                  uint16_t     validBits,
                                  BitAlignment alignment)
{
    if (validBits != validBitsOf(format))
    {
        return false;
    }
    if (packing != requiredPackingOf(format))
    {
        return false;
    }
    // 本批只产出 LsbZeroPadded；另两个取值不可达，出现即拒绝。
    // 这不是"未知就失败"的运行期分支，而是**拒绝一个本批不实现的取值**。
    return alignment == BitAlignment::LsbZeroPadded;
}

/// 由格式推出**本批**要求的载荷必要性策略（§4.1 第 2 条）：
/// 有效位深 > 8 ⇒ `RawRequired`（显示图是加工产物）；否则 `RawOptional`。
constexpr RawDataPolicy requiredRawPolicyOf(PixelFormat format)
{
    return validBitsOf(format) > 8 ? RawDataPolicy::RawRequired
                                   : RawDataPolicy::RawOptional;
}

/// 按紧凑契约计算期望载荷长度，**以 64 位做乘法并检查溢出**。
///
/// ⚠ 溢出检查不是防御性编程的摆设：`width`/`height` 是设备回报的
/// 32 位值，两个 4e9 相乘即溢出 64 位无符号数，此时若不检查就会
/// 得到一个"看起来很小"的期望长度，进而通过长度校验，然后按这个
/// 长度去 memcpy —— 这正是"先校验、再复制"要挡住的形态。
///
/// @return false 表示：格式本批不支持、或乘法溢出（调用方判 `CorruptFrame`）。
inline bool computeExpectedCompactBytes(uint32_t    width,
                                        uint32_t    height,
                                        PixelFormat format,
                                        uint64_t&   outBytes)
{
    const CompactLayout layout = compactLayoutOf(format);
    if (!layout.supported || layout.bytesNumerator == 0 ||
        layout.bytesDenominator == 0)
    {
        return false;
    }

    // 先按 "像素数 × 分子" 检查，再除分母（Mono12Packed 的 3/2）。
    // 顺序不能颠倒：先除会把 Mono12Packed 的奇数像素结果截断。
    const uint64_t pixels = static_cast<uint64_t>(width) *
                            static_cast<uint64_t>(height);
    const uint64_t kMaxU64 = ~static_cast<uint64_t>(0);
    if (pixels != 0 && kMaxU64 / pixels < layout.bytesNumerator)
    {
        return false;   // 溢出
    }

    const uint64_t scaled = pixels * layout.bytesNumerator;
    outBytes              = scaled / layout.bytesDenominator;
    return true;
}

/// 一帧原始载荷及其**全部**解释信息（ENG-09 V2.3 §5.28）。
///
/// ⚠ 长度与布局的关系（本批的核心纪律）：
///   `sdkPayloadBytes`    设备自报的**唯一权威总长**（`IMV_FrameInfo::size`）
///   `expectedCompactBytes` 按紧凑契约算出的期望长度
///   `compactSizeMatches` 两者是否相等 —— **只是一个校验条件**
/// ⚠ 该布尔量**不得**改名为"布局已确认"之类：一次字节数比较只能说明
/// "**长度与紧凑契约相容**"，**证明不了像素怎么排列**（相同字节数
/// 对应多种排列是可能的）。把校验条件说成结论，就会让下游以为
/// "布局已被验证过"，从而放心地按某个未经验证的假设去寻址。
struct RawImagePayload
{
    /// 自有字节，**逐字节等于 SDK 交付的缓冲**（复制在 `IMV_ReleaseFrame`
    /// 之前完成，见文件头）。空指针表示"本帧未携带原始载荷"。
    std::shared_ptr<const std::vector<uint8_t>> bytes;

    /// 项目定义的格式（由 SDK 格式码经适配层映射表得出）。
    PixelFormat format = PixelFormat::Mono8;

    /// 有效位数：8 / 12。
    uint16_t validBits = 8;

    /// 打包方式。本批只支持 `Unpacked`。
    Packing packing = Packing::Unpacked;

    /// 有效位对齐。本批恒为 `LsbZeroPadded`（PFNC 2.4 §6.1.1），
    /// 随元数据一同落盘，使离线复现时不必回查本批的实现代码。
    BitAlignment bitAlignment = BitAlignment::LsbZeroPadded;

    /// 设备回报的宽高（不是配置里的期望值）。
    uint32_t width  = 0;
    uint32_t height = 0;

    /// SDK 自报的载荷总长（`IMV_FrameInfo::size`），**唯一权威长度**。
    uint64_t sdkPayloadBytes = 0;

    /// 按本批紧凑契约算出的期望长度；溢出或格式不支持时为 0。
    uint64_t expectedCompactBytes = 0;

    /// = (`sdkPayloadBytes == expectedCompactBytes`)，见上方的命名纪律。
    /// 为假 ⇒ 该帧不符合本批契约 ⇒ `OpStatus::CorruptFrame`，**不发布帧**。
    bool compactSizeMatches = false;

    /// 多字节样本的组装顺序，**适配层声明**（依据见 ENG-09 V2.3 §5.28 第 4 条）。
    /// ⚠ 8 位 BMP 写出**不能**证明 16 位容器的字节序（BMP 体只吃
    /// 8 位字节流），故本字段的取值连同其依据一起登记为实机核实项。
    ByteOrder declaredByteOrder = ByteOrder::LittleEndian;

    // ── 以下为 SDK 原值，**仅作诊断**（语义未经确认，禁止用于任何寻址或
    //    长度计算）。它们随错误记录与日志一并留存，使"长度不符"这类
    //    可查事实在离线排查时不被丢掉。
    int32_t sdkPixelFormatCode = 0;  ///< SDK 返回的像素格式码原值
    int32_t sdkPaddingX        = 0;  ///< 未文档化语义，禁止寻址
    int32_t sdkPaddingY        = 0;  ///< 未文档化语义，禁止寻址
};

}  // namespace data
}  // namespace aircraft
