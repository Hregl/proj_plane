// ============================================================================
//  src/application/AlignmentController.cpp
//
//  依据：SYS-10 §7 / §8 / §14、SYS-08 §5.4 / §7.7、ENG-09 §2.2 / §2.4 / §5.10
// ============================================================================

#include "application/AlignmentController.h"

#include <cmath>
#include <string>

namespace aircraft
{
namespace application
{

namespace
{

/// 弧度 → 度的换算因子。ENG-09 §2.2 冻结角度单位为度，并说明需要弧度时
/// "在算法内部局部转换"，不得改变结构体字段单位。SYS-10 §8 的 arctan 结果
/// 是弧度，故必须在此转成度再写入命令。
constexpr double kRadToDeg = 57.295779513082320876798154814105;

/// 判定行程上下限是否**已配置**。
///
/// `TurntableConfig` 的四个限位字段默认都是 0（ENG-09 §6.3 无默认值）。
/// 若把"上下限都是 0"当作真实行程，则任何非零命令都会被判越程并报 2002 ——
/// 一次根本没配 yaml 的启动会表现为"转台一动就超程报警"，
/// 而真实原因是配置缺失。故要求 min < max 才启用越程判定；
/// 不满足时**跳过判定并在 message 中记录**（不静默，同
/// CameraSynchronizer 对未配置同步容差的处理）。
bool limitsConfigured(double lo, double hi)
{
    return hi > lo;
}

}  // namespace

// ---------------------------------------------------------------------------

AlignmentController::AlignmentController(const data::TurntableConfig& config)
    : config_(config)
    , lastError_()
    , commandValid_(true)
{
}

data::TurntableCommand AlignmentController::calculate(
    const data::TargetOffset& offset,
    const data::TurntableState& current,
    const data::CameraCalibration& calibration)
{
    // ---- 失败时的统一返回：保持当前角度 ----
    //
    // 不用"夹取到限位"再返回：夹取后的命令看起来是合法的，调用方一旦下发，
    // 转台就会真的移动到限位上。而 §7.7 对越程的要求是"**停止运动** +
    // 报警 + FAILED"。返回"当前位置"使最粗心的调用方也不会引起运动。
    const data::TurntableCommand hold{
        current.azimuth,
        current.elevation};

    // ---- 前提 ①：标定内参可用 ----
    //
    // 需要的是 cameraMatrix 的 fx / fy（单位 **像素**）。
    // ⚠ 不能用 CameraChannel::focalLength：ENG-09 §2.3 冻结它为**米**
    //（100 mm 镜头写作 0.1）。用米代入 arctan(Δpixel / f) 会得到
    // Δθ ≈ arctan(2000/0.1) ≈ 89.997°，转台直接飞向行程端点。
    // 这是本项目最容易发生且最不容易被发现的一处量纲错误。
    if (calibration.cameraMatrix.empty()
        || calibration.cameraMatrix.rows < 3
        || calibration.cameraMatrix.cols < 3
        || calibration.cameraMatrix.type() != CV_64F)
    {
        commandValid_ = false;
        // 4001（裁决 C-006）：此前这一族"标定无效"路径借用裸 0，而 0 的
        // 冻结语义是"未设置"、在 result.json 中渲染为 "OK" —— 于是一台
        // **因缺标定而失败**的测量，其终态错误码看起来像一切正常。
        // 本类中另外三处同族拒绝（fx/fy 无效、缺图像尺寸、内参不可信）
        // 同样置 4001。
        lastError_ = data::ErrorInfo{
            data::kErrCalibrationMissing,
            "标定内参不可用（cameraMatrix 为空或非 3x3 CV_64F），无法换算角度",
            0};
        return hold;
    }

    const double fx = calibration.cameraMatrix.at<double>(0, 0);
    const double fy = calibration.cameraMatrix.at<double>(1, 1);

    if (!(fx > 0.0) || !(fy > 0.0)
        || !std::isfinite(fx) || !std::isfinite(fy))
    {
        commandValid_ = false;
        lastError_ = data::ErrorInfo{
            data::kErrCalibrationMissing,
            "标定内参无效：fx/fy 必须为有限正数（当前 fx=" + std::to_string(fx)
                + ", fy=" + std::to_string(fy) + "）",
            0};
        return hold;
    }

    // ---- 前提 ①b：图像尺寸必须已知（像素偏差的参考原点要求）----
    //
    // ENG-09 §2.4 冻结"像素坐标相对图像中心"，故本类收到的 pixelX/pixelY
    // 只有配上靶面尺寸才有物理含义。缺图像尺寸的标定 = **未标定**。
    // 这不是格式洁癖：data::CameraCalibration 的默认构造给出的正是
    // 一个 3x3 单位阵 + imageWidth = imageHeight = 0，即 fx = fy = **1**。
    // fx = 1 代入 arctan(Δpixel / f) 会得到一个"算得出来"的巨大角度
    //（Δpixel=200 → 89.68°），而它看上去完全合法 —— 转台会真的朝那个
    // 方向大幅转动。所以必须在这里拦住。
    if (calibration.imageWidth <= 0 || calibration.imageHeight <= 0)
    {
        commandValid_ = false;
        lastError_ = data::ErrorInfo{
            data::kErrCalibrationMissing,
            "标定不含图像尺寸（imageWidth/imageHeight 未填），无法确认像素偏差"
            "的参考原点，按未标定处理并拒绝换算",
            0};
        return hold;
    }

    // ---- 前提 ①c：视场必须小于 90°（fx > 半幅宽）----
    //
    // 判据取自几何本身，不是经验阈值：fx ≤ imageWidth/2 等价于水平视场
    // ≥ 90°，即针孔模型已退化到无法定位目标；俯仰同理。
    // 本系统三支镜头（25/50/100 mm，2048 宽靶面）的 fx 在 10³~10⁴ 像素
    // 量级，与此界相差两个数量级以上，故正常标定永不会触发；能触发它的
    // 只有占位内参（例如只填了 imageWidth 而 fx 仍为 1 的半成品标定）。
    // 与 ①b 同理：宁可拒绝并报警，也不下发一条"看起来合理"的大角度指令。
    if (!(fx > static_cast<double>(calibration.imageWidth) / 2.0)
        || !(fy > static_cast<double>(calibration.imageHeight) / 2.0))
    {
        commandValid_ = false;
        lastError_ = data::ErrorInfo{
            data::kErrCalibrationMissing,
            "标定内参不可信：fx=" + std::to_string(fx) + ", fy="
                + std::to_string(fy) + " 相对于靶面 " + std::to_string(
                    calibration.imageWidth) + "x"
                + std::to_string(calibration.imageHeight)
                + " 意味着视场 ≥ 90°，只可能来自占位内参",
            0};
        return hold;
    }

    // ---- 前提 ②：输入的像素偏差与当前角度可用 ----
    if (!std::isfinite(offset.pixelX) || !std::isfinite(offset.pixelY)
        || !std::isfinite(current.azimuth) || !std::isfinite(current.elevation))
    {
        commandValid_ = false;
        lastError_ = data::ErrorInfo{
            data::kErrStateFailure,
            "输入含非有限值（像素偏差或转台当前角度为 NaN/Inf）",
            0};
        return hold;
    }

    // ---- SYS-10 §8：Δθ = arctan(Δpixel / f)，结果为弧度 ----
    const double dAzRad = std::atan(offset.pixelX / fx);
    const double dElRad = std::atan(offset.pixelY / fy);

    // ---- 符号约定（ENG-09 §2.4 明确要求显式处理 Y 的符号）----
    //
    // 图像坐标：X 向右为正，Y **向下**为正（与 OpenCV 一致）。
    //  · 方位：目标在图像中心右侧（pixelX > 0）→ 相机须向右转 → 方位角增大。
    //  · 俯仰：目标在图像中心**下方**（pixelY > 0）→ 相机须**向下**转。
    //    俯仰角按 SYS-10 §3.2 的口径以"向上为正"（工作范围 -60° ~ +60°，
    //    水平为零位），故向下转 = 俯仰角**减小**，取负号。
    //
    // ⚠ 这两个符号在冻结文档中**没有明文给出**（SYS-10 §8 只给幅值公式）。
    // 取上述约定是因为它是唯一与 ENG-09 §2.4 的"Y 向下为正"自洽的一种；
    // 若方向取反，闭环会**发散**而不是静止 —— 表现为对准次数耗尽报 2003。
    // 现场首次接线后必须实测确认一次，并回填 SYS-10 §8（ENG-09 §8 流程）。
    const double azimuth   = current.azimuth
                           + dAzRad * kRadToDeg;
    const double elevation = current.elevation
                           - dElRad * kRadToDeg;

    // ---- 越程判定（§7.7：能力边界，不重试，FAILED 2002）----
    const bool azLimited = limitsConfigured(config_.azimuthMin, config_.azimuthMax);
    const bool elLimited = limitsConfigured(config_.elevationMin, config_.elevationMax);

    if (azLimited
        && (azimuth < config_.azimuthMin || azimuth > config_.azimuthMax))
    {
        commandValid_ = false;
        lastError_ = data::ErrorInfo{
            data::kErrTurntableOverTravel,
            "方位命令 " + std::to_string(azimuth) + "° 超出转台行程 ["
                + std::to_string(config_.azimuthMin) + ", "
                + std::to_string(config_.azimuthMax) + "]°",
            0};
        return hold;
    }

    if (elLimited
        && (elevation < config_.elevationMin || elevation > config_.elevationMax))
    {
        commandValid_ = false;
        lastError_ = data::ErrorInfo{
            data::kErrTurntableOverTravel,
            "俯仰命令 " + std::to_string(elevation) + "° 超出转台行程 ["
                + std::to_string(config_.elevationMin) + ", "
                + std::to_string(config_.elevationMax) + "]°",
            0};
        return hold;
    }

    data::TurntableCommand cmd;
    cmd.azimuthCommand = azimuth;
    cmd.elevationCommand = elevation;

    commandValid_ = true;

    if (!azLimited || !elLimited)
    {
        // 行程未配置（或只配了一轴）：命令已算出但**未做越程保护**。
        // 如实记录，使"限位没生效"成为可见事实而非静默假设。
        lastError_ = data::ErrorInfo{
            0,
            std::string("转台行程未完整配置（")
                + (azLimited ? "" : "方位 ")
                + (elLimited ? "" : "俯仰 ")
                + "限位 min==max，跳过越程判定）",
            0};
    }
    else
    {
        lastError_ = data::ErrorInfo{};
    }

    return cmd;
}

// ---------------------------------------------------------------------------

bool AlignmentController::isCentered(const data::TargetOffset& offset) const
{
    // SYS-08 §5.4：目标中心误差 ≤ ±50 pixel；SYS-10 §7 给出判据形式，
    // 阈值来自 TurntableConfig::centerThreshold（ENG-09 §6.3，默认 50）。
    const double t = config_.centerThreshold;
    return std::fabs(offset.pixelX) <= t && std::fabs(offset.pixelY) <= t;
}

void AlignmentController::judgeCentered(data::TargetOffset& offset) const
{
    offset.centered = isCentered(offset);
}

bool AlignmentController::commandValid() const
{
    return commandValid_;
}

data::ErrorInfo AlignmentController::lastError() const
{
    return lastError_;
}

bool AlignmentController::overTravel() const
{
    return lastError_.code == data::kErrTurntableOverTravel;
}

}  // namespace application
}  // namespace aircraft
