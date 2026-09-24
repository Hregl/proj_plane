// ============================================================================
//  src/optical/CoordinateTransformer.cpp
//
//  依据：ENG-09 §2.1 / §2.2 / §5.23 / §5.24、SYS-13 §5 / §6
// ============================================================================

#include "optical/CoordinateTransformer.h"

#include <cmath>

namespace aircraft
{
namespace optical
{

namespace
{
constexpr double kPi     = 3.14159265358979323846;
constexpr double kDegPerRad = 180.0 / kPi;

/// 判断矩阵元素是否全部为有限值。
///
/// 必要性：上游若出现除零（例如焦距为 0 的未装载标定），
/// 旋转/平移矩阵中会静默出现 NaN 或 Inf。NaN 参与后续比较时
/// **恒为 false**，于是所有阈值判据（yaw ∈ [yawMin, yawMax]、
/// 重投影误差 <= 上限）都会"不通过"，表现为验证失败但看不出根因；
/// 更糟的是 NaN 传播到输出后，序列化成 JSON 会得到 "nan"，
/// 使离线工具无法解析。
/// 在此处一次性拦下，让"数据里出现了非有限值"成为显式失败。
bool allFinite(const cv::Matx33d& m)
{
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            if (!std::isfinite(m(r, c)))
            {
                return false;
            }
        }
    }
    return true;
}

/// 判断旋转矩阵是否近似正交（R·Rᵀ ≈ I）。
///
/// 必要性：若外参文件里的 rotation 填的是欧拉角而不是旋转矩阵
/// （格式误会），矩阵仍为 3x3 且元素有限，但分解出的欧拉角毫无意义。
/// 正交性是旋转矩阵的**定义性质**，据此可以廉价地拦住这类输入。
///
/// 容差 1e-3：标定数据经浮点运算与十进制往返后，正交性偏差通常在
/// 1e-6 量级；1e-3 足以容纳这些误差，又能拦住"填错成欧拉角"这类
/// 偏差量级为 1 的输入。
bool isRotation(const cv::Matx33d& m, double tol = 1e-3)
{
    const cv::Matx33d rrt = m * m.t();
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            const double expected = (r == c) ? 1.0 : 0.0;
            if (std::fabs(rrt(r, c) - expected) > tol)
            {
                return false;
            }
        }
    }
    return true;
}
}  // namespace

CoordinateTransformer::CoordinateTransformer(const CalibrationManager& calibration)
    : calibration_(calibration)
{
}

data::Transform CoordinateTransformer::cameraToRig(data::CameraRole role) const
{
    return calibration_.getCalibration(role).cameraToRig;
}

data::ShipPoseResult CoordinateTransformer::transform(
    data::CameraRole        role,
    const data::CameraPose& cameraPose) const
{
    data::ShipPoseResult result;

    const data::CameraCalibration calib = calibration_.getCalibration(role);

    // ---- 输入校验 ----
    if (!allFinite(cameraPose.aircraftToCamera.rotation) ||
        !allFinite(calib.cameraToRig.rotation) ||
        !allFinite(calibration_.getRigToShip().rotation))
    {
        result.success = false;
        return result;
    }

    if (!isRotation(cameraPose.aircraftToCamera.rotation) ||
        !isRotation(calib.cameraToRig.rotation) ||
        !isRotation(calibration_.getRigToShip().rotation))
    {
        result.success = false;
        return result;
    }

    // ---- 坐标链合成（ENG-09 §2.1）----
    //
    //     aircraftToShip = rigToShip · cameraToRig · aircraftToCamera
    //
    // ⚠ 顺序：最靠近被变换点的变换在最右。见头文件的记忆要点。
    const data::Transform rigToShip = calibration_.getRigToShip();

    const cv::Matx33d rotation = rigToShip.rotation
                               * calib.cameraToRig.rotation
                               * cameraPose.aircraftToCamera.rotation;

    // 平移项按同一顺序复合：
    //     t_ship = t_rigToShip + R_rigToShip · ( t_cameraToRig
    //              + R_cameraToRig · t_aircraftToCamera )
    // 直接照抄旋转的乘法顺序来算平移是**错的** —— 平移不是矩阵乘，
    // 必须逐级把上一级的结果转入下一级坐标系。
    const cv::Vec3d tAfterRig = calib.cameraToRig.translation
                              + calib.cameraToRig.rotation
                                * cameraPose.aircraftToCamera.translation;
    const cv::Vec3d tShip = rigToShip.translation + rigToShip.rotation * tAfterRig;

    result.aircraftToShip.rotation    = rotation;
    result.aircraftToShip.translation = tShip;

    // ---- 欧拉角分解 ----
    double yawDeg   = 0.0;
    double pitchDeg = 0.0;
    double rollDeg  = 0.0;

    if (!decomposeZyx(rotation, yawDeg, pitchDeg, rollDeg))
    {
        // 分解失败（万向锁或非旋转矩阵）：整个结果不可用。
        // 不返回"部分正确"的结果 —— 一个 yaw 正确而 roll 错误的姿态
        // 在下游会与正确的姿态无法区分（下游只看 yaw）。
        result.success = false;
        return result;
    }

    result.yaw   = yawDeg;    // deg
    result.pitch = pitchDeg;  // deg
    result.roll  = rollDeg;   // deg

    // 重投影误差随姿态一并前传（ENG-09 §5.23 → §5.24）：
    // 它是 VALIDATE 状态的判据之一（SYS-07 §12.2），
    // 若在此处丢弃，验证阶段就只能重新解算一次才能拿到它。
    result.reprojectionError = cameraPose.reprojectionError;

    result.success = true;
    return result;
}

bool CoordinateTransformer::decomposeZyx(const cv::Matx33d& r,
                                         double& yawDeg,
                                         double& pitchDeg,
                                         double& rollDeg)
{
    if (!allFinite(r) || !isRotation(r))
    {
        return false;
    }

    // ZYX 顺序（ENG-09 §5.24：Yaw → Pitch → Roll）：
    //     R = Rz(yaw) · Ry(pitch) · Rx(roll)
    // 展开后第三行第一列恒为 −sin(pitch)，故：
    //     pitch = asin(−R(2,0))
    //     yaw   = atan2(R(1,0), R(0,0))
    //     roll  = atan2(R(2,1), R(2,2))
    const double r20 = r(2, 0);

    // 万向锁：|R(2,0)| = 1 时 pitch = ±90°，yaw 与 roll 退化为
    // 只由二者之差（或和）决定，无法分别确定。
    //
    // ⚠ 处理方式是**明确失败**而不是"取 yaw=0 然后算 roll"：
    //   后者会给出一个数值上自洽、但与真实姿态不同的解。
    //   对本系统而言，出现在 pitch = ±90° 附近的姿态本身就超出了
    //   舰载飞机的实际姿态范围（SYS-13 的 yaw 限定在 ±30°），
    //   因此这里更可能是输入异常（矩阵填错）而非真实工况，
    //   明确失败比编造一个解更安全。
    constexpr double kGimbalLockThreshold = 1.0 - 1e-9;
    if (std::fabs(r20) >= kGimbalLockThreshold)
    {
        return false;
    }

    const double pitchRad = std::asin(-r20);
    const double yawRad   = std::atan2(r(1, 0), r(0, 0));
    const double rollRad  = std::atan2(r(2, 1), r(2, 2));

    yawDeg   = yawRad   * kDegPerRad;
    pitchDeg = pitchRad * kDegPerRad;
    rollDeg  = rollRad  * kDegPerRad;

    // 单位是 deg（ENG-09 §2.2）。
    // ⚠ 不要在此处换算成弧度 —— 全工程角度一律为度，
    //   §2.2 举出的反面例子正是 `if (yaw <= 1.0)` 这类判断：
    //   若实际是弧度，判据的物理含义会差 3437 倍。
    return true;
}

}  // namespace optical
}  // namespace aircraft
