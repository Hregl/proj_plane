// ============================================================================
//  src/infrastructure/recorder/Recorder.cpp
//
//  result.json 手写而非用 JSON 库：
//    本工程无 JSON 依赖（现场一体机离线部署，依赖越少越好），而这里
//    要写的结构是固定的十来个字段。手写的唯一风险是字符串未转义 ——
//    由 escapeJson() 统一处理，且所有写入点都经它。
//
//  ⚠ 数值一律以 6 位小数写入（角度/比例）或 9 位有效数字（矩阵元素）：
//    写全 double 的 17 位有效数字会让文件难以肉眼判读，而 6 位小数
//    对 1 角分（= 0.0167°）的判读已有两个数量级余量。
// ============================================================================

#include "infrastructure/recorder/Recorder.h"

#include <cstdio>
#include <fstream>
#include <sstream>

#include <opencv2/core.hpp>

#include "data/ErrorInfo.h"
#include "data/FailureTrace.h"
#include "data/MeasurementRecord.h"
#include "data/MeasurementState.h"
#include "data/TurntableState.h"
#include "infrastructure/FileUtil.h"
#include "infrastructure/logger/Logger.h"

namespace aircraft
{
namespace infrastructure
{

namespace
{

using fileutil::joinPath;
using fileutil::makeDirectories;

/// JSON 字符串转义（" 与 \ 与 < 0x20 的控制字符）。
/// 本工程写入的字符串只有 taskId / 版本号 / 枚举名，正常不含特殊字符；
/// 但 modelId 可能来自配置文件（用户可填任意字符串），故必须转义。
std::string escapeJson(const std::string& in)
{
    std::string out;
    out.reserve(in.size() + 8);
    for (const char c : in)
    {
        switch (c)
        {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n";  break;
        case '\r': out += "\\r";  break;
        case '\t': out += "\\t";  break;
        default:
            if (static_cast<unsigned char>(c) < 0x20)
            {
                char buf[8] = {0};
                std::snprintf(buf, sizeof(buf), "\\u%04x",
                              static_cast<unsigned>(c));
                out += buf;
            }
            else
            {
                out += c;
            }
        }
    }
    return out;
}

std::string num(double v, int decimals)
{
    // ⚠ 非有限值一律写成 0，**不**原样交给 snprintf。
    //
    // 直接写会产出 `nan` / `inf`，而它们**不是合法 json** ——
    // 于是整份 result.json 变成语法错误的文件：任何用 json 解析器读它的
    // 下游都直接失败，**整份结果都无法回放**（不是少一个字段，是整份读不
    // 出来）。而 `save()` 仍返回 true —— 从落盘一侧看，这份坏包与一份好包
    // 没有任何区别。
    //
    // 为什么不是 close 的 `null`（json 里表达"没有这个数"的标准写法）：
    // **OpenCV 4.6 的 json 解析器不支持 null**（实测报
    // `Value 'null' is not supported by this parser`），而 OpenCV 正是
    // 本工程读取结果包的解析器。写成字符串 `"nan"` 更糟：`>> double`
    // 会把它读成 1.79e308（实测），比语法错误更难发现。
    //
    // 故选择"写成 0 + 在 result.json 的 `non_finite_fields` 里点名"：
    // 文件永远可解析，且"这个字段原本不是一个数"是**机器可读**的事实，
    // 而不是一个看起来正常的 0。与 `missing_required_fields` 同一手法。
    //
    // 可达性：非有限值确实能走到这里。`PoseValidator` 的各项判据都是
    // `x > 上限` / `x < 下限` 形式的比较，而 NaN 与任何数比较都为 false，
    // 故 yaw = NaN 的姿态**不会被判不合格**，会一路走到 SAVE 并落盘。
    // （"该由哪一层拦住非有限姿态"是待裁决项，见 README §6。）
    if (!std::isfinite(v))
    {
        return std::string("0");
    }

    char buf[64] = {0};
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
    return std::string(buf);
}

/// 测量状态名。与 application/StateMachine.cpp、MeasurementController.cpp、
/// ui/UiText.h 中的同名函数逐字一致。
///
/// ⚠ 这是**第四份**同样的枚举名函数（前三份分别在上述三处）。重复的理由
///   与前三份相同：infrastructure 不能引用 application（反向依赖），
///   而 data/MeasurementState.h 里没有 name 函数（ENG-01 §5 的文件清单
///   未含它，新增属清单变更）。四份副本之间的漂移风险是真实的
///   （result.json 里的状态名会与界面、日志里的不一致），已登记待裁决：
///   是否在 data 层冻结 measurementStateName()。
const char* stateName(data::MeasurementState state)
{
    switch (state)
    {
    case data::MeasurementState::IDLE:           return "IDLE";
    case data::MeasurementState::SEARCH:         return "SEARCH";
    case data::MeasurementState::TARGET_FOUND:   return "TARGET_FOUND";
    case data::MeasurementState::ALIGN:          return "ALIGN";
    case data::MeasurementState::STABILIZE:      return "STABILIZE";
    case data::MeasurementState::MEASURE_SELECT: return "MEASURE_SELECT";
    case data::MeasurementState::CAPTURE:        return "CAPTURE";
    case data::MeasurementState::POSE_SOLVE:     return "POSE_SOLVE";
    case data::MeasurementState::VALIDATE:       return "VALIDATE";
    case data::MeasurementState::SAVE:           return "SAVE";
    case data::MeasurementState::COMPLETE:       return "COMPLETE";
    case data::MeasurementState::FAILED:         return "FAILED";
    }
    return "UNKNOWN";
}

/// 通道名。与 application/MeasurementController.cpp 的同名函数逐字一致
/// （`optical/CalibrationManager.cpp` 里还有一份局部的字符串表）。
///
/// ⚠ 这是**第三份** `CameraRole → 名字` 的映射。与上面的 `stateName()`
///   是同一类问题的同一类成因：infrastructure 不得依赖 application
///   （ENG-01 §18），而 data 层没有冻结的 name 函数。漂移的后果在这里
///   更具体：`result.json` 里写 "CAM100"、界面与日志里写别的拼法时，
///   现场按通道排查会查到两个不同的名字。已与 `stateName()` 一并登记
///   （见其上方说明：是否在 data 层冻结这些枚举名函数）。
const char* roleName(data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM25:  return "CAM25";
    case data::CameraRole::CAM50:  return "CAM50";
    case data::CameraRole::CAM100: return "CAM100";
    }
    return "UNKNOWN_ROLE";
}

/// ---- 原始载荷的四项解释信息（011-A1；ENG-09 V2.4 §5.28 / §6.5）----
///
/// ⚠ 四个函数全部落盘用**名字**，与 motionName / roleName 同一条理由：
///   枚举值落盘后，将来在中间插入一个成员会让历史包里的 "1" 悄悄改含义，
///   而 `cam*.raw` 是**无头裸缓冲** —— 它唯一的解码依据就是这些名字。
///   一个含义漂移的格式名会让历史原始数据**按错误布局解码**，
///   而结果仍然"看起来是一张图"（灰度系统性偏移，不报错）。
///
/// ⚠ 这四张表**目前只有本文件一个消费者**（全仓没有 result.json 的读回
///   实现）。故不放进 data 层：那属于"是否在 data 冻结枚举名函数"这一
///   未裁决事项（见上方 stateName / roleName 的说明），本批不替它预先
///   决定。将来出现第二个消费者时，再按那时的裁决合并。
const char* pixelFormatName(data::PixelFormat format)
{
    switch (format)
    {
    case data::PixelFormat::Mono8:        return "Mono8";
    case data::PixelFormat::Mono12:       return "Mono12";
    case data::PixelFormat::Mono12Packed: return "Mono12Packed";
    case data::PixelFormat::BGR8:         return "BGR8";
    }
    return "UNKNOWN_FORMAT";
}

const char* packingName(data::Packing packing)
{
    switch (packing)
    {
    case data::Packing::Unpacked: return "Unpacked";
    case data::Packing::Packed12: return "Packed12";
    }
    return "UNKNOWN_PACKING";
}

const char* bitAlignmentName(data::BitAlignment alignment)
{
    switch (alignment)
    {
    case data::BitAlignment::LsbZeroPadded: return "LsbZeroPadded";
    case data::BitAlignment::MsbAligned:    return "MsbAligned";
    case data::BitAlignment::Unknown:       return "Unknown";
    }
    return "UNKNOWN_ALIGNMENT";
}

const char* byteOrderName(data::ByteOrder order)
{
    switch (order)
    {
    case data::ByteOrder::LittleEndian: return "LittleEndian";
    case data::ByteOrder::BigEndian:    return "BigEndian";
    }
    return "UNKNOWN_BYTE_ORDER";
}

/// `cam*.raw` 的内容来源：本批三分支（ENG-09 V2.4 §5.28 第 7 条）。
enum class RawSource
{
    None,          ///< 该路本轮没有可用帧 ⇒ **不写文件**（不是写 0 字节）
    RawBytes,      ///< 写**原始载荷本体**（`raw.bytes`，逐字节等于 SDK 缓冲）
    DisplayImage   ///< 写显示图 —— **仅**当 8 位格式且 raw 缺失时允许
};

/// 三路原始文件的**唯一**文件名清单（下标与 role 一一对应：0=CAM25）。
///
/// ⚠ 单列一份而不是在两个函数里各写一遍字面量：这两处会**同时**出现在
///   同一条契约错误的文本里（`writeResultJson` 先写、`writeRawFrames`
///   随后），两份字面量一旦漂移，现场拿到的通道名就可能与实际写出的
///   文件名对不上 —— 而那正是"按文件排查"的人唯一的抓手。
const char* const kRawFileNames[3] = {"cam25.raw", "cam50.raw", "cam100.raw"};

const char* dataSourceName(RawSource source)
{
    switch (source)
    {
    case RawSource::None:         return "none";
    case RawSource::RawBytes:     return "raw";
    case RawSource::DisplayImage: return "image";
    }
    return "UNKNOWN_SOURCE";
}

/// 某一路写什么 —— **单一实现**（011-A1 §3.3）。
///
/// ⚠ 为什么必须是一个函数、被两处调用：`writeResultJson` 写元数据、
///   `writeRawFrames` 写文件，两者必须对"这一路到底写的是什么"给出
///   **同一个答案**。各写一份必然在某次修改后分叉，而分叉的形态极难察觉：
///   元数据说 `data_source = "raw"`、文件里其实是 8U 显示图 ——
///   离线解码时按 16 位容器去读一张 8 位图，得到的仍"是一张图"。
///
/// ⚠ 判据**只依据帧自身携带的字段**（`raw` / `rawPolicy` / `captureFormat`），
///   不靠文件名、逻辑 `cameraId`，也不在落盘时重查配置 —— 后者会让
///   "结果包说自己是什么"依赖于"哪个配置恰好在场"。
///
/// @param contractError 非空 ⇒ 该帧违反本批契约，**调用方必须让本包失败**：
///        既不写文件，也不得把它当成"这一路本轮没有帧"（那会把一次
///        契约违背降级成一次正常的缺图）。
RawSource decideRawSource(const data::ImageFrame& frame, std::string& contractError)
{
    contractError.clear();
    const data::RawImagePayload& raw    = frame.raw;
    const bool                   hasRaw = raw.bytes && !raw.bytes->empty();

    if (hasRaw)
    {
        // ---- 复检（后端发布帧时已断言过一次，此处**不信任上游**）----
        //  这里不是"重复劳动"：帧可能来自别的后端实现、别的版本，
        //  也可能在测试里被手工拼出来。落盘是最后一道能拦住它的地方。
        //
        //  ⚠ 011-A1 九项缺口 §6：所谓"不信任上游"在上一版其实**不成立** ——
        //    它只读了上游算好的两个字段（`compactSizeMatches` 与
        //    `expectedCompactBytes`），**从不重算**，于是上游把期望长度
        //    算错（或干脆填错）时，这里会原样放行并把错误的元数据写进
        //    结果包。判据必须只依据**帧自身的字段**：重算而得，而不是
        //    转录别人的结论。
        if (raw.format != frame.captureFormat)
        {
            contractError = "raw.format（" +
                            std::string(pixelFormatName(raw.format)) +
                            "）与帧的 captureFormat（" +
                            std::string(pixelFormatName(frame.captureFormat)) +
                            "）不一致 —— 二者必须同源";
            return RawSource::None;
        }
        if (!data::isValidCombination(raw.format, raw.packing, raw.validBits,
                                      raw.bitAlignment))
        {
            // ⚠ 这一条同时把 `bitAlignment != LsbZeroPadded` 挡在外面
            //   （`MsbAligned` / `Unknown` 本批不实现）—— 对齐方式决定
            //   12 位样本在 16 位容器里的位置，按错的假设解码是"整幅图
            //   看起来正常、数值整体偏了 4 位"这种查不出来的错。
            contractError = "原始载荷的组合不合法：format=" +
                            std::string(pixelFormatName(raw.format)) + " packing=" +
                            std::string(packingName(raw.packing)) + " validBits=" +
                            std::to_string(raw.validBits) + " alignment=" +
                            std::string(bitAlignmentName(raw.bitAlignment));
            return RawSource::None;
        }

        // ---- 策略与格式一致：`rawPolicy` 必须由 `captureFormat` 推出 ----
        // ⚠ 上一版只查了 `rawPolicy` 本身是不是 `RawRequired`（在"无 raw"
        //   的分支里），**从不查它与格式是否相符** ⇒ 一个
        //   `captureFormat=Mono8` 而 `rawPolicy=RawRequired` 的帧（或反之）
        //   会被当作自洽的帧保存，于是"这一路该不该有载荷"这个判断在结果
        //   包里失去了可信来源。两个字段必须给出同一个答案。
        const data::RawDataPolicy requiredPolicy =
            data::requiredRawPolicyOf(frame.captureFormat);
        if (frame.rawPolicy != requiredPolicy)
        {
            contractError =
                "帧的 rawPolicy（" +
                std::string(frame.rawPolicy == data::RawDataPolicy::RawRequired
                                ? "RawRequired"
                                : "RawOptional") +
                "）与 captureFormat（" +
                std::string(pixelFormatName(frame.captureFormat)) +
                "，有效位 " +
                std::to_string(data::validBitsOf(frame.captureFormat)) +
                "）不符：该格式要求 " +
                std::string(requiredPolicy == data::RawDataPolicy::RawRequired
                                ? "RawRequired"
                                : "RawOptional") +
                " —— 二者必须由同一条规则推出";
            return RawSource::None;
        }

        // ---- 字节序：本批只声明小端 ----
        // ⚠ `PixelFormat.h` 定义了 `BigEndian` 但**全仓此前没有任何一处
        //   读过它**（即"声明为哪种字节序"这件事此前完全不参与判断）。
        //   本批的实现只声明小端，故大端声明一出现就是"这份载荷不是按
        //   我们实现的规则组装的" —— 按小端去读会得到逐样本字节颠倒的值，
        //   而图仍是"一张合法的图"。
        if (raw.declaredByteOrder != data::ByteOrder::LittleEndian)
        {
            contractError =
                "原始载荷声明为 " +
                std::string(byteOrderName(raw.declaredByteOrder)) +
                "，本批适配层只声明 LittleEndian（依据见 ENG-09 §5.28 第 4 条）"
                " —— 按小端解码这份载荷会逐样本字节颠倒";
            return RawSource::None;
        }

        // ---- 长度：**重算**，并让四个事实互证 ----
        //
        // 四个事实（缺一不可，全部走 64 位）：
        //   ① 重算值   —— 由 `width × height × 布局` 当场算出（**不转录**）
        //   ② `raw.expectedCompactBytes` —— 上游算出的期望长度
        //   ③ `raw.sdkPayloadBytes`      —— 设备自报的权威总长
        //   ④ `raw.bytes->size()`        —— 载体实际大小（**写出去的就是它**）
        // 四者必须相等：①②不等说明上游的期望值算错；②③不等说明这一帧
        // 根本不符合紧凑契约；③④不等说明元数据描述的长度与文件不符 ——
        // 那正是"按元数据解码得到一张尺寸不对的图"的来源。
        uint64_t recomputedBytes = 0;
        if (!data::computeExpectedCompactBytes(raw.width, raw.height, raw.format,
                                              recomputedBytes))
        {
            contractError = "无法按紧凑契约重算期望长度（width=" +
                            std::to_string(raw.width) + " height=" +
                            std::to_string(raw.height) + " format=" +
                            std::string(pixelFormatName(raw.format)) +
                            "）：格式本批不支持，或乘法溢出 64 位";
            return RawSource::None;
        }

        const uint64_t carrierBytes = static_cast<uint64_t>(raw.bytes->size());
        if (recomputedBytes != raw.expectedCompactBytes ||
            raw.expectedCompactBytes != raw.sdkPayloadBytes ||
            raw.sdkPayloadBytes != carrierBytes)
        {
            contractError =
                "原始载荷的长度四方不一致（重算=" +
                std::to_string(recomputedBytes) + "，expected_compact_bytes=" +
                std::to_string(raw.expectedCompactBytes) +
                "，sdk_payload_bytes=" + std::to_string(raw.sdkPayloadBytes) +
                "，载体=" + std::to_string(carrierBytes) + "；width=" +
                std::to_string(raw.width) + " height=" +
                std::to_string(raw.height) + " format=" +
                std::string(pixelFormatName(raw.format)) +
                "）—— 四个数字必须相等，否则这份载荷的解码依据是错的";
            return RawSource::None;
        }

        // ---- `compactSizeMatches` 与上述事实**不得矛盾** ----
        // 到这里四个事实已经一致（即按契约"长度相符"），故该布尔量必须为真。
        // 它为假时**不是**"长度不符"（那已被上面拦下），而是"上游算出的
        // 布尔量与它自己的两个字段矛盾" —— 本批契约要求这类帧不被发布，
        // 故落盘是最后一道拦截：**不得**忽略它再照原样保存错误元数据。
        if (!raw.compactSizeMatches)
        {
            contractError =
                "compact_size_matches=false 与重算结果矛盾（重算／期望／自报／"
                "载体 四者均为 " +
                std::to_string(recomputedBytes) +
                " 字节）—— 该布尔量由 sdk_payload_bytes 与 "
                "expected_compact_bytes 推出，二者相等时它必为真";
            return RawSource::None;
        }
        return RawSource::RawBytes;
    }

    // ---- 原始载荷缺失：分支二（照常保存）还是分支三（拒绝回落）----
    //
    //  判据是**帧的两个字段**，不是"图是不是空的"：
    //    · `rawPolicy == RawRequired`（有效位深 > 8 的格式本批即 Mono12）
    //    · 或 `captureFormat` 本身的有效位深 > 8
    //  两个都查：任一为真即拒绝。只查一个的话，"rawPolicy 被留成默认值
    //  RawOptional 而没有跟着 captureFormat 走"这一种错误组合就会漏过 ——
    //  而那恰恰是 `[缺口 1]` 点名的情形（12 位帧丢了载荷，文件里却留着一张
    //  8 位图，事后无法分辨）。
    if (frame.rawPolicy == data::RawDataPolicy::RawRequired ||
        data::validBitsOf(frame.captureFormat) > 8)
    {
        contractError =
            "该路声明为 " + std::string(pixelFormatName(frame.captureFormat)) +
            "（有效位 " + std::to_string(data::validBitsOf(frame.captureFormat)) +
            "，rawPolicy=" +
            (frame.rawPolicy == data::RawDataPolicy::RawRequired ? "RawRequired"
                                                                 : "RawOptional") +
            "）却**未携带原始载荷**：显示图是加工产物，回落保存等于把"
            "不可复原的数据当成测量结果（禁止静默存 8 位图）";
        return RawSource::None;
    }

    if (!frame.image.empty())
    {
        // 显示图必须真的是 8U：下面的字节数是按 `elemSize()` 算的，
        // 而元数据里的 `pixel_format` 与 `valid_bits` 描述的是这个文件。
        // 一个 16U/32F 的显示图会让"元数据描述的文件布局"与文件不符。
        if (frame.image.depth() != CV_8U)
        {
            contractError = "显示图的深度不是 8U（depth=" +
                            std::to_string(frame.image.depth()) +
                            "）—— 本批只把 8U 显示图当作可保存的回落内容";
            return RawSource::None;
        }

        // ---- 通道数必须与**已校验的格式**相符（011-A1 九项缺口 §6）----
        //
        // ⚠ 上一版只查了 `depth() == CV_8U`，于是**通道数完全不参与判断**
        //    ⇒ 一份 `captureFormat = Mono8`、`image` 却是 3 通道 BGR 的帧
        //    会被写成 `pixel_format: "Mono8"` 配 `rows × cols × 3` 字节的
        //    文件：元数据说单通道，文件里是三通道。离线按元数据解码得到的
        //    是一张"宽度是对的、每行只取前 1/3"的错位图 —— 而它看起来
        //    完全像一张正常的图。
        //
        // 判据取自 `captureFormat` 而**不是** `image.channels()`：要问的是
        // "这份文件符合它声称的格式吗"，不是"它自己说自己是什么"。
        // 本批能走到这里的格式只有 8 位两种（>8 位的已在上面被拒绝），
        // 故映射是纯查表：Mono8 → 1，BGR8 → 3。
        const int expectedChannels =
            (frame.captureFormat == data::PixelFormat::BGR8) ? 3 : 1;
        if (frame.image.channels() != expectedChannels)
        {
            contractError =
                "显示图的通道数（" + std::to_string(frame.image.channels()) +
                "）与 captureFormat（" +
                std::string(pixelFormatName(frame.captureFormat)) + "，应为 " +
                std::to_string(expectedChannels) +
                " 通道）不符 —— 元数据将描述这份文件，二者必须一致"
                "（否则离线按元数据解码会得到一张错位的图）";
            return RawSource::None;
        }
        return RawSource::DisplayImage;
    }

    // 该路本轮没有可用帧（`capture()` 每轮从零构造 frame，故这里
    // 不会拿到上一轮的旧图）。不写文件，由元数据的 data_source = "none" 表达。
    return RawSource::None;
}

/// 转台运动状态名（`turntable.json` 与 result.json 共用）。
///
/// ⚠ 落盘用**名字**而不是枚举的整数值：整数值会随枚举成员顺序变化而
///   悄悄改变含义（在中间插入一个成员，历史结果包里的 "2" 就从
///   STABLE 变成别的），而名字不会。这是与 ENG-09 §5.27 禁止裸错误码
///   同一条理由在**落盘格式**上的应用。
const char* motionName(data::TurntableMotionState motion)
{
    switch (motion)
    {
    case data::TurntableMotionState::IDLE:   return "IDLE";
    case data::TurntableMotionState::MOVING: return "MOVING";
    case data::TurntableMotionState::STABLE: return "STABLE";
    case data::TurntableMotionState::ERROR:  return "ERROR";
    }
    return "UNKNOWN_MOTION";
}

/// 验证不合格的原因名（裁决 C-008）。
///
/// ⚠ 与 `stateName()` / `roleName()` 落盘用**名字**而不是整数值同一条理由
///   （见 motionName 的说明）：枚举成员顺序变化会让历史结果包里的 `"1"`
///   悄悄改变含义。
///
/// ⚠ 目前**只有这一份** `PoseValidationReason → 名字` 的映射（实测：ui/ 的
///   任何文件都还没有引用 `PoseValidationResult`）。写在这里是因为
///   result.json 需要一个机读的原因名。
///   但它注定要变成第二份：界面迟早要把"为什么判不合格"显示给人看，
///   而 `ui` 与 `infrastructure` 之间**不能互相引用**（ENG-01 §18），
///   届时那份映射只能另写一遍 —— 与 `stateName()` / `roleName()` 是
///   同一类成因。届时应连同它们一起，在 data 层冻结 name 函数（已登记）。
///   **不要**在 ui 里写第二份而又不留登记。
const char* validationReasonName(data::PoseValidationReason reason)
{
    switch (reason)
    {
    case data::PoseValidationReason::OK:               return "OK";
    case data::PoseValidationReason::NON_FINITE_VALUE: return "NON_FINITE_VALUE";
    }
    return "UNKNOWN_REASON";
}

std::string jsonBool(bool v)
{
    return v ? std::string("true") : std::string("false");
}

/// 一个 `ErrorInfo` 的 json 对象：`code` + `name` + `message` + `timestamp_ns`。
///
/// ⚠ 同时写 `code` 与 `name`，不省任何一个：
///   `name` 是 ENG-09 §5.27 要求的对外形式（离线排查按名字检索），
///   而 `code` 是唯一能做**区间筛选**的依据 —— "哪些是 5xxx 的模型段故障"
///   用名字是筛不出来的（名字里没有段信息，且将来改名的代价远大于加一个字段）。
///   只写其一会让这三类查询各废掉一类。
///
/// ⚠ `code == 0` 一并如实写出，渲染为 "OK"（`errorCodeName(0)`）。
///   这不是"空字段"，而是"此处没有错误"这一事实本身 ——
///   history 里正常前进的迁移靠它把"失败引起的迁移"与"正常迁移"分开
///   （见 StateTransition.h 的 error 说明）。
///
/// @param indent 当前行的缩进（嵌套对象与闭括号按它对齐）。
void writeErrorObject(std::ostream& os, const data::ErrorInfo& e,
                      const std::string& indent)
{
    os << "{\n";
    os << indent << "  \"code\": " << e.code << ",\n";
    os << indent << "  \"name\": \"" << data::errorCodeName(e.code) << "\",\n";
    os << indent << "  \"message\": \"" << escapeJson(e.message) << "\",\n";
    os << indent << "  \"timestamp_ns\": " << e.timestampNs << "\n";
    os << indent << "}";
}

/// 简单字符串的 YAML 标量（用于 config_snapshot 的只读副本 —— 那是
/// 原样拷贝，不需要转义）。
bool copyFile(const std::string& from, const std::string& to)
{
    std::ifstream in(from, std::ios::binary);
    if (!in.is_open())
    {
        return false;
    }
    std::ofstream out(to, std::ios::binary | std::ios::trunc);
    if (!out.is_open())
    {
        return false;
    }
    out << in.rdbuf();
    out.flush();
    return out.good();
}

}  // namespace

// ---------------------------------------------------------------------------

Recorder::Recorder(const data::SystemConfig& system,
                   std::string configDir,
                   std::string calibrationId,
                   std::string modelId,
                   std::string featureVersion)
    : outputDir_(system.outputDir)
    , configDir_(std::move(configDir))
    , calibrationId_(std::move(calibrationId))
    , modelId_(std::move(modelId))
    , featureVersion_(std::move(featureVersion))
{
}

std::string Recorder::lastResultPath() const
{
    if (lastPackageDir_.empty())
    {
        return std::string();
    }
    return joinPath(lastPackageDir_, "result.json");
}

// ---------------------------------------------------------------------------

bool Recorder::writeResultJson(const data::MeasurementRecord& record,
                              const std::string& packageDir,
                              std::string& error)
{
    const data::MeasurementTask&      task = record.task;
    const data::ShipPoseResult&       pose = task.result;
    const data::PoseValidationResult& val  = task.validation;

    // SYS-04 §6.4 必含字段中**确实拿不到**的那些。
    //
    // ⚠ 这张表已从 6 项缩到 0~1 项：入参改为 MeasurementRecord 之后，
    // camera_used / selected_camera / score / quality / feature_count /
    // match_count 全部有了承载者（这正是本记录存在的理由之一）。
    // 现在唯一可能进表的只剩 model_type —— C-003 的
    // synthetic / production 机型库尚未实施时它的值为空，
    // 而这**必须**作为"结果包不完整"暴露出来，而不是静默写一个空串：
    // 空串在 json 里看起来完全正常，却让"这份结果能不能用"无从判断。
    std::vector<std::string> missing;
    if (record.modelType.empty())
    {
        missing.push_back("model_type");
    }

    // ---- 非有限值点名表 ----
    //
    // `num()` 会把非有限值写成 0（理由见其说明：json 表达不了"没有数"，
    // 而 OpenCV 又不认 null）。若到此为止，一个 NaN 姿态在结果包里
    // 就是一个**看起来完全正常的 0.000000** —— 正是本项目反复出现的
    // "字段存在、类型合法、数值看起来正常，但语义不成立"。
    // 故把它逐项点名，让"这个数不是数"变成机器可读的事实。
    //
    // 检查范围与 `num()` 的调用点一致：所有可能承载**测量结果**的浮点量。
    // 整数计数（feature_count 等）与图像尺寸不可能非有限，故不列。
    std::vector<std::string> nonFinite;
    const auto checkFinite = [&nonFinite](const std::string& key, double v)
    {
        if (!std::isfinite(v))
        {
            nonFinite.push_back(key);
        }
    };

    checkFinite("yaw", pose.yaw);
    checkFinite("pitch", pose.pitch);
    checkFinite("roll", pose.roll);
    checkFinite("reprojection_error", pose.reprojectionError);
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            checkFinite("aircraftToShip.rotation[" + std::to_string(r) + "]["
                            + std::to_string(c) + "]",
                        pose.aircraftToShip.rotation(r, c));
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        checkFinite("aircraftToShip.translation[" + std::to_string(i) + "]",
                    pose.aircraftToShip.translation[i]);
    }
    checkFinite("validation.reprojection_error", val.reprojectionError);
    checkFinite("validation.inlier_ratio", val.inlierRatio);
    checkFinite("validation.confidence", val.confidence);
    checkFinite("score", record.selectedScore);
    checkFinite("quality.sharpness", record.selectedQuality.sharpness);
    checkFinite("quality.exposure", record.selectedQuality.exposure);
    checkFinite("quality.contrast", record.selectedQuality.contrast);
    checkFinite("quality.match_ratio", record.selectedQuality.matchRatio);
    checkFinite("statistics.match_ratio", record.statistics.matchRatio());
    checkFinite("statistics.spread_px", record.statistics.spreadPx);
    checkFinite("turntable.azimuth", record.turntable.azimuth);
    checkFinite("turntable.elevation", record.turntable.elevation);

    std::ostringstream os;
    os << "{\n";
    os << "  \"task_id\": \"" << escapeJson(task.taskId) << "\",\n";
    os << "  \"state\": \"" << stateName(task.state) << "\",\n";
    os << "  \"written_at\": \"" << Logger::wallClockText() << "\",\n";
    os << "  \"model_id\": \"" << escapeJson(record.modelId) << "\",\n";
    os << "  \"model_type\": \"" << escapeJson(record.modelType) << "\",\n";
    os << "  \"feature_version\": \"" << escapeJson(featureVersion_) << "\",\n";
    os << "  \"calibration_id\": \"" << escapeJson(record.calibrationId) << "\",\n";
    os << "  \"software_version\": \"" << escapeJson(record.softwareVersion)
       << "\",\n";
    os << "  \"success\": " << jsonBool(pose.success) << ",\n";
    os << "  \"yaw\": " << num(pose.yaw, 6) << ",\n";
    os << "  \"pitch\": " << num(pose.pitch, 6) << ",\n";
    os << "  \"roll\": " << num(pose.roll, 6) << ",\n";
    os << "  \"reprojection_error\": " << num(pose.reprojectionError, 6) << ",\n";

    // aircraftToShip（ENG-09 §5.24）：3×3 旋转 + 3 平移。
    // 旋转矩阵按行展开，便于与 OpenCV 的 Matx33d 直接比对。
    const data::Transform& tf = pose.aircraftToShip;
    os << "  \"aircraftToShip\": {\n";
    os << "    \"rotation\": [\n";
    for (int r = 0; r < 3; ++r)
    {
        os << "      [" << num(tf.rotation(r, 0), 9) << ", "
           << num(tf.rotation(r, 1), 9) << ", " << num(tf.rotation(r, 2), 9)
           << "]" << (r < 2 ? "," : "") << "\n";
    }
    os << "    ],\n";
    os << "    \"translation\": [" << num(tf.translation[0], 6) << ", "
       << num(tf.translation[1], 6) << ", " << num(tf.translation[2], 6)
       << "]\n";
    os << "  },\n";

    os << "  \"validation\": {\n";
    os << "    \"valid\": " << jsonBool(val.valid) << ",\n";
    // 判不合格的**原因分类**（裁决 C-008）。与三个数值并列写出：
    // 原因回答"属不属于需要特殊处置的那一类"（目前只有非有限值一类），
    // 数值回答"差多少" —— 而 `NON_FINITE_VALUE` 时后面的数值全都不可信。
    os << "    \"reason\": \"" << validationReasonName(val.reason) << "\",\n";
    os << "    \"reprojection_error\": " << num(val.reprojectionError, 6) << ",\n";
    os << "    \"inlier_ratio\": " << num(val.inlierRatio, 6) << ",\n";
    os << "    \"confidence\": " << num(val.confidence, 6) << "\n";
    os << "  },\n";

    // ---- 通道选择（SYS-04 §6.4 的 camera_used / selected_camera / score）----
    //
    // ⚠ 同时写出 `degraded` 与 `cameras_available`：只有"选了 100 mm"
    // 而无"当时有 几路可用"时，读者无法判断这次选择是在什么条件下做出的 ——
    // 2 路降级时选中 100 mm 与 3 路正常时选中 100 mm，
    // 是两件可信度完全不同的事。
    os << "  \"camera_used\": \"" << roleName(record.selectedCamera) << "\",\n";
    os << "  \"selected_camera\": \"" << roleName(record.selectedCamera)
       << "\",\n";
    os << "  \"score\": " << num(record.selectedScore, 6) << ",\n";
    os << "  \"degraded\": " << jsonBool(record.degraded) << ",\n";
    os << "  \"cameras_available\": " << record.camerasAvailable << ",\n";

    // ---- 质量（SYS-04 §6.4 的 quality）----
    //
    // ⚠ 这些是**被选中那一次采集**的质量，与本次解算所用的图像**同源**
    // （裁决 C-002 / V2.1-C02 §1.1 D-C02-1）。修复之前二者是不同帧，
    // 记录里的质量描述的是一张没被解算的图，且不报错。
    const data::ImageQuality& q = record.selectedQuality;
    os << "  \"quality\": {\n";
    os << "    \"sharpness\": " << num(q.sharpness, 6) << ",\n";
    os << "    \"exposure\": " << num(q.exposure, 6) << ",\n";
    os << "    \"contrast\": " << num(q.contrast, 6) << ",\n";
    os << "    \"feature_count\": " << q.featureCount << ",\n";
    os << "    \"match_count\": " << q.matchCount << ",\n";
    os << "    \"match_ratio\": " << num(q.matchRatio, 6) << "\n";
    os << "  },\n";

    // ---- 匹配统计（SYS-04 §6.4 的 feature_count / match_count）----
    //
    // ⚠ 这是 `statistics`（算法链推送，裁决 C-002），与上面的 `quality`
    // **不是同一组数**：`quality.match_count` 是**内点数**（过 RANSAC 之后），
    // 本节的 `match_count` 是**融合后保留的对应数**（尚未过 RANSAC）。
    // 二者同名而不同义，故在此**并列写出**，让读者一眼看到这是两组值
    // 而不是重复字段。详见 data/MeasurementStatistics.h 的字段说明。
    os << "  \"statistics\": {\n";
    os << "    \"feature_count\": " << record.statistics.featureCount << ",\n";
    os << "    \"match_count\": " << record.statistics.matchCount << ",\n";
    os << "    \"match_ratio\": " << num(record.statistics.matchRatio(), 6)
       << ",\n";
    os << "    \"dropped_by_conflict\": " << record.statistics.droppedByConflict
       << ",\n";
    os << "    \"cad_count\": " << record.statistics.cadCount << ",\n";
    os << "    \"texture_count\": " << record.statistics.textureCount << ",\n";
    os << "    \"spread_px\": " << num(record.statistics.spreadPx, 6) << "\n";
    os << "  },\n";

    // ---- 被解算的那一次采集（D-C02-2：raw 与 result.json 必须同源）----
    //
    // ⚠ 011-A1 起，`cam*.raw` 的**解码依据**是每路的
    //   `data_source` + `pixel_format` / `valid_bits` / `packing` /
    //   `valid_bit_alignment` / `declared_byte_order`，
    //   **不是** `display_image` 里的 width/height/type ——
    //   `display_image` 描述的是**显示图**（12 位格式时是右移后的加工产物），
    //   它的尺寸与类型解释不了原始载荷。旧版把 `image.type()` 当作裸缓冲的
    //   解码依据，是"12 位数据被按 8 位读"这条静默失效的来源（见
    //   writeRawFrames 的说明与 RawImagePayload.h 的文件头）。
    //
    // ⚠ 每个文件的**文件名**由 role 决定（CAM25 → cam25.raw），下标 0/1/2
    //   与 role 一一对应；`data_source == "none"` 表示该文件**不存在**。
    //
    // ⚠ 元数据的键大多与 `RawImagePayload` 的字段一一对应（共 12 个），
    //   外加 `raw_image`（011-A1 九项缺口 §4）与 `display_image`：
    //   与 writeRawFrames 用**同一个** decideRawSource，故"元数据说写的是什么"
    //   与"文件里实际是什么"不可能分叉。
    //
    // ⚠ 011-A1 九项缺口 §4：**`raw_image` 是裸缓冲自己的宽高**。
    //   上一版只有 `display_image.width/height`（取自 `image.cols/rows`），
    //   而 `RawImagePayload::width/height`（设备回报的原始尺寸）**全文件
    //   从不读取** —— 于是 ① 12 位帧可以"有原始载荷、没有显示图"，
    //   那种包里宽高是 0，**无法独立恢复原始二维图像**；② 即便显示图在，
    //   它也是**加工产物**（12 位时是右移 4 位的结果），拿它当载荷的几何
    //   依据是"用显示图解释原始载荷"的同一类错误。
    //   ∴ 只有在 `data_source == "raw"`（真身路径）时才写 `raw_image`：
    //   回落路径（`data_source == "image"`）**没有**原始载荷，写 0 会让人
    //   以为"原始图是 0×0"，而 `data_source` 已经如实表明了来源 ——
    //   不写这个键，比写一个会被误读的键好。
    //
    //   裸缓冲的**完整**解码依据因此是：
    //     `pixel_format` / `valid_bits` / `packing` / `valid_bit_alignment` /
    //     `declared_byte_order` ＋ **`raw_image.width/height`**（真身路径），
    //   或（回落路径）`pixel_format` ＋ `display_image.width/height`。
    //   `display_image` 仅供人看与离线预览，**不是**真身路径的解码依据。
    os << "  \"best_frame\": {\n";
    os << "    \"exposure_index\": " << record.bestFrame.exposureIndex << ",\n";
    os << "    \"trigger_timestamp_ns\": " << record.bestFrame.triggerTimestamp
       << ",\n";
    os << "    \"frames\": [\n";
    const data::ImageFrame* chans[3] = {&record.bestFrame.cam25,
                                        &record.bestFrame.cam50,
                                        &record.bestFrame.cam100};
    for (int i = 0; i < 3; ++i)
    {
        std::string     contractError;
        const RawSource source = decideRawSource(*chans[i], contractError);
        if (!contractError.empty())
        {
            // 与 writeRawFrames 同一条判据、同一段文本（含同一个文件名，
            // 取自 kRawFileNames）：result.json 先写，故这里先把它拦下来 ——
            // 一份"元数据已落盘、raw 文件却没写"的结果包是最坏的一种
            // （它看起来完整）。
            error = "结果包的原始载荷不合法（" + std::string(kRawFileNames[i]) +
                    "）：" + contractError;
            return false;
        }

        const data::RawImagePayload& raw = chans[i]->raw;

        // ---- 元数据描述的是**实际写出的那个文件**（§6）----
        //
        // ⚠ 上一版无论哪条分支都从 `raw` 取格式/有效位/打包/对齐 ——
        //   而回落分支（`data_source == "image"`）**根本没有 raw**，
        //   取到的是 `RawImagePayload` 的**默认字段**。默认值恰好是
        //   `Mono8/8/Unpacked`，于是"`captureFormat=BGR8`、写出去的
        //   是 3 通道图"这种帧会被描述成 `Mono8` 的单通道文件 ——
        //   元数据与文件自相矛盾，且矛盾的方向会让人按 1 字节/像素去读
        //   一条 3 字节/像素的行。
        //   ∴ 分支各自给出**该文件自己的**描述：
        //     真身路径 → `raw` 的字段（刚刚已由 decideRawSource 四方校验）
        //     回落路径 → 已校验的 `captureFormat`（+ `image` 的通道数）
        const bool isRawBytes = (source == RawSource::RawBytes);
        const data::PixelFormat fileFormat =
            isRawBytes ? raw.format : chans[i]->captureFormat;
        const uint16_t fileValidBits =
            isRawBytes ? raw.validBits : data::validBitsOf(fileFormat);
        const data::Packing filePacking =
            isRawBytes ? raw.packing : data::requiredPackingOf(fileFormat);
        const data::BitAlignment fileAlignment =
            isRawBytes ? raw.bitAlignment : data::bitAlignmentOf(fileFormat);
        // 回落路径的 8 位样本**逐字节**写出，不存在多字节样本的组装顺序
        // ⇒ 该字段对这份文件不适用；此处仍写声明值只是让键集保持稳定
        // （与 `bitAlignmentOf` 对 8 位格式仍返回一个确定取值的处理同构），
        // 解码方**不得**据它为 8 位文件推断任何组装方式。
        const data::ByteOrder fileByteOrder =
            isRawBytes ? raw.declaredByteOrder : data::ByteOrder::LittleEndian;

        os << "      {\"role\": \"" << roleName(chans[i]->role)
           << "\", \"frame_id\": " << chans[i]->frameId
           << ", \"timestamp_ns\": " << chans[i]->timestampNs
           // 该路本轮是否交付了可用帧（= 该路的 cam*.raw 是否写出）。
           // ⚠ 判据取自三分支判定，**不再**等同于 `!image.empty()`：
           //   12 位帧可以"有原始载荷、没有显示图"，那仍是可用帧。
           << ", \"frame_valid\": " << jsonBool(source != RawSource::None)
           << ",\n"
           // 显示图（**仅供人看与离线预览**，不是 raw 的解码依据）。
           // 空图时 width/height 为 0，本身即"没有显示图"的表达。
           << "       \"display_image\": {\"width\": " << chans[i]->image.cols
           << ", \"height\": " << chans[i]->image.rows
           << ", \"type\": " << chans[i]->image.type() << "},\n"
           << "       \"data_source\": \"" << dataSourceName(source) << "\",\n";

        // ---- `raw_image`：**仅在真身路径**写出（§4）----
        // 回落路径的宽高由 `display_image` 给出（那份文件就是显示图），
        // 写一个 `raw_image` 会让人以为存在一份原始载荷。
        if (isRawBytes)
        {
            os << "       \"raw_image\": {\"width\": " << raw.width
               << ", \"height\": " << raw.height << "},\n";
        }

        // ⚠ 键集**按数据来源分组**（011-A1 缺口 6 的收尾项，本批补齐）：
        //   回落路径（`data_source == "image"`）**没有**原始载荷，也没有
        //   任何一次 SDK 调用产生过它的元数据，故下列事实**一律不写**：
        //     `sdk_payload_bytes` / `expected_compact_bytes` /
        //     `compact_size_matches` / `sdk_pixel_format_code` /
        //     `sdk_padding_x` / `sdk_padding_y`。
        //   上一版只给 `raw_image` 做了这件事，这六个键却照样写出，
        //   值还是 `raw` 的**默认值**（0／false）—— 读包的人会看到
        //   "载荷 0 字节、长度相符=false"配着一份 512 字节的显示图文件，
        //   与 §4 给 `raw_image` 的理由（"写 0 会误导"）**完全同构**。
        //   "不存在的键"胜过"值不对的键"：缺键一眼看得出没有这项事实，
        //   而 `0` 会被当成一个测出来的数字。
        //   ∴ 真身路径写全（它刚刚已由 `decideRawSource` 四方校验），
        //     回落路径只写**这份文件自己**的描述（来自已校验的
        //     `captureFormat` 与 `image`）。
        os << "       \"pixel_format\": \"" << pixelFormatName(fileFormat)
           << "\", \"valid_bits\": " << fileValidBits
           << ", \"packing\": \"" << packingName(filePacking)
           << "\",\n"
           << "       \"valid_bit_alignment\": \""
           << bitAlignmentName(fileAlignment) << "\",\n";

        if (isRawBytes)
        {
            os << "       \"sdk_payload_bytes\": " << raw.sdkPayloadBytes
               << ", \"expected_compact_bytes\": " << raw.expectedCompactBytes
               << ",\n"
               << "       \"compact_size_matches\": "
               << jsonBool(raw.compactSizeMatches) << ",\n";
        }

        os << "       \"declared_byte_order\": \""
           << byteOrderName(fileByteOrder) << "\"";

        if (isRawBytes)
        {
            os << ",\n"
               << "       \"sdk_pixel_format_code\": " << raw.sdkPixelFormatCode
               << ", \"sdk_padding_x\": " << raw.sdkPaddingX
               << ", \"sdk_padding_y\": " << raw.sdkPaddingY;
        }

        os << "}" << (i < 2 ? "," : "") << "\n";
    }
    os << "    ]\n";
    os << "  },\n";

    // ---- 转台（turntable.json 另存一份，此处为内联摘要）----
    os << "  \"turntable\": {\n";
    os << "    \"azimuth\": " << num(record.turntable.azimuth, 6) << ",\n";
    os << "    \"elevation\": " << num(record.turntable.elevation, 6) << ",\n";
    os << "    \"motion\": \"" << motionName(record.turntable.motion) << "\"\n";
    os << "  },\n";

    // ---- 失败根因与终点（裁决 C-007）----
    //
    // ⚠ 为什么在 result.json 里只留摘要、全量轨迹另存 failure.json：
    //   result.json 是**每次测量都被读**的那个文件（界面、离线工具、
    //   按 task 检索），而 history 的条数随重试次数增长，且它的用途
    //   （复原路径、算各状态停留时长）是**故障复盘**时才有的。把两者
    //   分开，读结果的人不必为一串与他无关的迁移付出解析成本。
    //   两份文件同源同一次生成（都取自 record.failure 这一个冻结值），
    //   不存在"摘要与轨迹漂移"的机会。
    //
    // ⚠ 任务成功时**照写不省**：此时 first_error.code == 0（渲染为 "OK"），
    //   整块看起来是"没有失败"。省略整块会让"这次成功"与"这份包来自
    //   C-007 之前的旧版本"在文件里长得一样 —— 而后者恰恰是没有根因
    //   信息的那种包，正是本裁决要消除的。
    //
    // ⚠ 字段顺序刻意与 FailureTrace 的成员声明顺序一致（先根因后终点），
    //   便于读文件时与代码逐行对照。
    const data::FailureTrace& fail = record.failure;
    os << "  \"failure\": {\n";
    os << "    \"first_failed_state\": \"" << stateName(fail.firstFailedState)
       << "\",\n";
    os << "    \"first_error\": ";
    writeErrorObject(os, fail.firstError, "    ");
    os << ",\n";
    os << "    \"final_failed_state\": \"" << stateName(fail.finalFailedState)
       << "\",\n";
    os << "    \"final_error\": ";
    writeErrorObject(os, fail.finalError, "    ");
    os << "\n";
    os << "  },\n";

    os << "  \"missing_required_fields\": [";
    for (std::size_t i = 0; i < missing.size(); ++i)
    {
        os << "\"" << escapeJson(missing[i]) << "\""
           << (i + 1 < missing.size() ? ", " : "");
    }
    os << "],\n";

    // 这些字段在文件里写的是 0，而 0 **不是**它们的值（见 num() 的说明）。
    // 正常任务下这张表为空 —— 非空就意味着本次测量的结果不可直接使用。
    os << "  \"non_finite_fields\": [";
    for (std::size_t i = 0; i < nonFinite.size(); ++i)
    {
        os << "\"" << escapeJson(nonFinite[i]) << "\""
           << (i + 1 < nonFinite.size() ? ", " : "");
    }
    os << "]\n";
    os << "}\n";

    const std::string path = joinPath(packageDir, "result.json");
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        error = "无法写入 " + path;
        return false;
    }
    out << os.str();
    out.flush();
    if (!out.good())
    {
        error = "写入失败（磁盘满？）：" + path;
        return false;
    }
    return true;
}

bool Recorder::writeFailureJson(const data::MeasurementRecord& record,
                               const std::string& packageDir,
                               std::string& error)
{
    // 失败任务的**全量**轨迹（裁决 C-007），与 result.json 的 failure 摘要同源。
    //
    // ⚠ 为什么这个文件自足（把 first / final 也再写一遍，而不是只写 history）：
    //   离线复盘的第一问永远是"根因是什么"，而复盘时打开的往往就是这一个
    //   文件。若根因只在 result.json 里，读 failure.json 的人必须同时打开
    //   另一个文件才能回答第一问 —— 于是实际使用中会退化成"只看 result.json"，
    //   而 history 又恰恰是 result.json 里没有的。两个文件都自足，
    //   代价是四个字段，收益是"打开哪个都能起步"。
    //   两者由**同一次** `record.failure` 生成，不会互相漂移。
    //
    // ⚠ history 按时间顺序原样写出，**不做任何聚合**（不省略、不合并同状态
    //   的连续条目、不排序）：顺序本身就是信息（"先 CAPTURE 失败还是先
    //   MEASURE_SELECT 失败"指向完全不同的处置方向，见 StateTransition.h）。
    const data::FailureTrace& fail = record.failure;

    std::ostringstream os;
    os << "{\n";
    os << "  \"first_failed_state\": \"" << stateName(fail.firstFailedState)
       << "\",\n";
    os << "  \"first_error\": ";
    writeErrorObject(os, fail.firstError, "  ");
    os << ",\n";
    os << "  \"final_failed_state\": \"" << stateName(fail.finalFailedState)
       << "\",\n";
    os << "  \"final_error\": ";
    writeErrorObject(os, fail.finalError, "  ");
    os << ",\n";
    os << "  \"history\": [\n";
    for (std::size_t i = 0; i < fail.history.size(); ++i)
    {
        const data::StateTransition& t = fail.history[i];
        os << "    {\n";
        os << "      \"from\": \"" << stateName(t.from) << "\",\n";
        os << "      \"to\": \"" << stateName(t.to) << "\",\n";
        os << "      \"timestamp_ns\": " << t.timestampNs << ",\n";
        os << "      \"error\": ";
        writeErrorObject(os, t.error, "      ");
        os << "\n";
        os << "    }" << (i + 1 < fail.history.size() ? "," : "") << "\n";
    }
    os << "  ]\n";
    os << "}\n";

    const std::string path = joinPath(packageDir, "failure.json");
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        error = "无法写入 " + path;
        return false;
    }
    out << os.str();
    out.flush();
    if (!out.good())
    {
        error = "写入失败（磁盘满？）：" + path;
        return false;
    }
    return true;
}

bool Recorder::writeRawFrames(const data::MultiCameraFrame& frame,
                             const std::string& packageDir,
                             std::string& error)
{
    // SYS-09 §12 要求结果包含 cam25|50|100.raw。
    //
    // ⚠ 格式选择（冻结文档未规定，此处明确并记录）：
    //   写**裸像素缓冲**（行连续，**不加文件头**）。
    //   裸缓冲单独存在时是不可解码的 —— 而"文件在、却打不开"比"文件不在"
    //   更糟（前者看起来一切正常）。故配套要求：
    //   ① result.json 的 best_frame.frames[] 里的 `pixel_format` /
    //      `valid_bits` / `packing` / `valid_bit_alignment` /
    //      `declared_byte_order` 是这份裸缓冲的**唯一**解码依据，
    //      缺了它们这份 raw 就没有意义；
    //   ② 三路一律按**该路实际**的尺寸写，不做统一缩放：
    //      三台相机分辨率本就可能不同，强行统一会在"哪一路被改过"
    //      这件事上制造一个事后查不出来的错误。
    //
    //  ⚠ 011-A1 的关键修正（ENG-09 V2.4 §5.28 第 6、7 条）：解码依据**不再**是
    //    `image` 的 `width/height/type`。`image` 是**显示图**（12 位格式时
    //    是右移后的加工产物），它的行列数与类型**不能**用来解释原始载荷；
    //    把 8U 显示图的 type 当作裸缓冲的解码依据，正是"12 位数据被按
    //    8 位读"这条静默失效的来源。
    //
    // ⚠ 三分支（判据在 decideRawSource，此处只负责落字节）：
    //    ① `raw` 非空 ⇒ 写真身（**逐字节等于 SDK 交付缓冲**）；
    //    ② `raw` 空、`RawOptional` 且 8 位 ⇒ 写显示图，标 data_source=image；
    //    ③ `raw` 空、但 `RawRequired` 或 12 位 ⇒ **契约错误，本包失败**
    //       （禁止静默存 8 位图）。
    //
    // ⚠ 没有可用帧的那一路**不写文件**，而不是写一个 0 字节文件：
    //   0 字节文件在目录列表里与"写失败被截断"无法区分。
    //   它的缺失由 result.json 中的 `data_source = "none"` 表达。
    const data::ImageFrame* chans[3] = {&frame.cam25, &frame.cam50,
                                        &frame.cam100};

    for (int i = 0; i < 3; ++i)
    {
        std::string      contractError;
        const RawSource  source = decideRawSource(*chans[i], contractError);
        if (!contractError.empty())
        {
            // 一路违反契约 ⇒ **整包失败**，理由同 save() 的既有原则：
            // 写一份"看起来完整、实际缺一路原始数据"的结果包，比不写更危险。
            // 该状态在生产路径上不可达（RawRequired 的帧必然带 raw），
            // 故它的出现本身就是代码缺陷指示 —— 错误文本必须点名通道与原因。
            error = "结果包的原始载荷不合法（" + std::string(kRawFileNames[i]) +
                    "）：" + contractError;
            return false;
        }

        const std::string path = joinPath(packageDir, kRawFileNames[i]);
        const char*       data = nullptr;
        std::size_t       bytes = 0;

        // 显示图分支才需要拷贝：子矩阵（ROI）不连续时直接按 rows*cols 写
        // 会按步长把别的像素写进来，得到一张"尺寸对、内容错"的图，
        // 而它看起来完全正常。原始载荷是一块平坦的 vector，天然连续。
        cv::Mat           contiguous;
        switch (source)
        {
        case RawSource::None:
            continue;
        case RawSource::RawBytes:
            data  = reinterpret_cast<const char*>(chans[i]->raw.bytes->data());
            bytes = chans[i]->raw.bytes->size();
            break;
        case RawSource::DisplayImage:
            contiguous = chans[i]->image.isContinuous() ? chans[i]->image
                                                        : chans[i]->image.clone();
            data  = reinterpret_cast<const char*>(contiguous.data);
            bytes = contiguous.total() * contiguous.elemSize();
            break;
        }

        std::ofstream out(path, std::ios::out | std::ios::binary
                                   | std::ios::trunc);
        if (!out.is_open())
        {
            error = "无法写入 " + path;
            return false;
        }
        out.write(data, static_cast<std::streamsize>(bytes));
        out.flush();
        if (!out.good())
        {
            error = "写入失败（磁盘满？）：" + path;
            return false;
        }
    }
    return true;
}

bool Recorder::writeTurntableJson(const data::TurntableState& turntable,
                                 const std::string& packageDir,
                                 std::string& error)
{
    // SYS-09 §12 的 turntable.json。与 result.json 里的同名字段**同源** ——
    // 保留两份是冻结文档对目录结构的要求，不是冗余；但必须**同一次取值**，
    // 故两者都由 `buildRecord()` 冻结下来的那一个 `record.turntable` 生成，
    // 不各自去问设备（否则两份文件可能相差一次转台动作）。
    const std::string path = joinPath(packageDir, "turntable.json");
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        error = "无法写入 " + path;
        return false;
    }
    out << "{\n";
    out << "  \"azimuth\": " << num(turntable.azimuth, 6) << ",\n";
    out << "  \"elevation\": " << num(turntable.elevation, 6) << ",\n";
    out << "  \"motion\": \"" << motionName(turntable.motion) << "\"\n";
    out << "}\n";
    out.flush();
    if (!out.good())
    {
        error = "写入失败（磁盘满？）：" + path;
        return false;
    }
    return true;
}

bool Recorder::writeConfigSnapshot(const std::string& packageDir, std::string& error)
{
    // SYS-04 §6.4 冻结：结果包必须保存本次生效的全部 config/*.yaml。
    // 理由：ValidationConfig 的阈值直接决定"结果是否有效"，
    // 回放时不带当时的阈值，同一批图像会得出不同判定。
    const std::string dir = joinPath(packageDir, "config_snapshot");
    if (!makeDirectories(dir))
    {
        error = "无法创建 " + dir;
        return false;
    }

    // 逐文件拷贝。缺失**不算失败但必须报告**：7 个文件里少一个，
    // 结果包的可复现性就已不完整，调用方须记进日志。
    static const char* const kFiles[] = {
        "system.yaml", "camera.yaml", "optical_rig.yaml", "turntable.yaml",
        "trigger.yaml", "measurement.yaml", "validation.yaml"};

    std::vector<std::string> missing;
    for (const char* f : kFiles)
    {
        const std::string from = joinPath(configDir_, f);
        const std::string to   = joinPath(dir, f);
        if (!copyFile(from, to))
        {
            missing.push_back(f);
        }
    }
    if (!missing.empty())
    {
        std::ostringstream os;
        os << "config_snapshot 缺少 " << missing.size() << " 个文件：";
        for (std::size_t i = 0; i < missing.size(); ++i)
        {
            os << missing[i] << (i + 1 < missing.size() ? ", " : "");
        }
        error = os.str();
        // 不是致命失败：结果本身仍然有效，只是复现性受损。
        // 返回 true 并让调用方把 error 写进日志。
        return true;
    }
    return true;
}

bool Recorder::writeCalibrationId(const std::string& packageDir, std::string& error)
{
    const std::string path = joinPath(packageDir, "calibration_id.txt");
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        error = "无法写入 " + path;
        return false;
    }
    out << calibrationId_ << "\n";
    out.flush();
    return out.good();
}

bool Recorder::save(const data::MeasurementRecord& record)
{
    lastErrorText_.clear();
    lastMissingFields_.clear();
    lastPackageDir_.clear();

    const data::MeasurementTask& task = record.task;

    // 目录名取 taskId（MeasurementController 已生成 "measurement_<ns>"，
    // 见其 taskId 注释）。taskId 为空时兜一个 —— 否则会写到 outputDir 根，
    // 把多个任务的结果混在一起。用墙钟可读形式便于现场辨认。
    std::string name = task.taskId;
    if (name.empty())
    {
        name = "measurement_unnamed_" + Logger::wallClockText();
        // 目录名里不能有空格与冒号
        for (char& c : name)
        {
            if (c == ' ' || c == ':')
            {
                c = '_';
            }
        }
    }

    const std::string dir = joinPath(outputDir_, name);
    if (!makeDirectories(dir))
    {
        lastErrorText_ = "无法创建结果目录：" + dir;
        return false;
    }
    lastPackageDir_ = dir;

    std::string error;
    bool        ok = true;

    // 各步**依次独立**执行，任一步失败都要把原因带出去；
    // 但失败后**不再继续**（写一个字段全空的结果包比不写更危险：
    // 它在目录里看起来是一份完整的测量结果）。
    if (!writeResultJson(record, dir, error))
    {
        lastErrorText_ = error;
        ok             = false;
    }
    // failure.json 紧接 result.json 写：两者是同一份 `record.failure` 的
    // 摘要与全量，落在相邻的两步里，中途不会有机会被别的数据源改写。
    // 它**对所有任务都写**（成功任务写的是"路径 + 无根因"），
    // 理由见 writeResultJson 中 failure 块与 writeFailureJson 的说明。
    if (ok && !writeFailureJson(record, dir, error))
    {
        lastErrorText_ = error;
        ok             = false;
    }
    if (ok && !writeRawFrames(record.bestFrame, dir, error))
    {
        lastErrorText_ = error;
        ok             = false;
    }
    if (ok && !writeTurntableJson(record.turntable, dir, error))
    {
        lastErrorText_ = error;
        ok             = false;
    }
    if (ok && !writeCalibrationId(dir, error))
    {
        lastErrorText_ = error;
        ok             = false;
    }
    if (ok && !writeConfigSnapshot(dir, error))
    {
        lastErrorText_ = error;
        ok             = false;
    }
    else if (ok && !error.empty() && lastErrorText_.empty())
    {
        // config_snapshot 不完整：结果有效但复现性受损，记在错误文本里
        // 供调用方写日志（返回 true）。
        lastErrorText_ = error;
    }

    // ⚠ 入参改用 MeasurementRecord 之后，这张表不再由 Recorder 单方面
    // 写死，而是**反映记录里实际为空**的必含字段（见 writeResultJson）。
    // 从前它是 6 项硬编码 —— 那 6 项恰恰是当时入参承载不了的字段，
    // 属于"编译期就知道会缺"，与"这次测量真的缺"是两回事。
    if (record.modelType.empty())
    {
        lastMissingFields_.push_back("model_type");
    }

    return ok;
}

}  // namespace infrastructure
}  // namespace aircraft
