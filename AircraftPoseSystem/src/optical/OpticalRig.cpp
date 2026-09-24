// ============================================================================
//  src/optical/OpticalRig.cpp
//
//  依据：ENG-09 §2.3（长度单位）、§3.3（裁决 C-18）、§5.4
//        ENG-01 §6、5.md §六
// ============================================================================

#include "optical/OpticalRig.h"

#include <algorithm>

namespace aircraft
{
namespace optical
{

namespace
{
/// 缺省三通道的焦距，单位 **m**（ENG-09 §2.3 冻结）。
///
/// ⚠ 这三个数值是 25/50/100 **毫米**镜头换算到米的结果，
///   不是工程口语里的 "25 / 50 / 100"。
///   5.md §六 直接写入 25.0 / 50.0 / 100.0（毫米值），
///   会使内参矩阵与以米为单位的三维点不匹配 —— 详见头文件的完整分析。
///
/// 用一个具名的毫秒→米换算常量表达，而不是直接写 0.025：
/// 让"这是从毫米换算来的"这一事实留在代码里。直接写 0.025 的读者
/// 无法判断它是笔误还是换算结果，而写 25.0/1000.0 则一眼可见。
constexpr double kMmToM = 1.0 / 1000.0;

/// 缺省镜头焦距（毫米，仅用于换算，不参与任何计算）。
constexpr double kDefaultFocalMm25  = 25.0;
constexpr double kDefaultFocalMm50  = 50.0;
constexpr double kDefaultFocalMm100 = 100.0;
}  // namespace

OpticalRig::OpticalRig() = default;

bool OpticalRig::initialize(const std::vector<data::CameraChannel>& channels)
{
    if (channels.empty())
    {
        // 缺省三通道：cameraId 用角色名，现场应由 cameras/*.yaml 覆盖
        // （ENG-01 §3.2），故这里的标识只保证"非空且唯一"。
        data::CameraChannel c25;
        c25.cameraId    = "CAM25";
        c25.role        = data::CameraRole::CAM25;
        c25.focalLength = kDefaultFocalMm25 * kMmToM;   // 0.025 m
        c25.enabled     = true;

        data::CameraChannel c50;
        c50.cameraId    = "CAM50";
        c50.role        = data::CameraRole::CAM50;
        c50.focalLength = kDefaultFocalMm50 * kMmToM;   // 0.05 m
        c50.enabled     = true;

        data::CameraChannel c100;
        c100.cameraId    = "CAM100";
        c100.role        = data::CameraRole::CAM100;
        c100.focalLength = kDefaultFocalMm100 * kMmToM;  // 0.1 m
        c100.enabled     = true;

        channels_ = {c25, c50, c100};
        return true;
    }

    channels_ = channels;

    // 每个角色必须**恰好**出现一次。
    //
    // 重复出现的后果：getCamera(role) 的返回值取决于表中顺序，
    // 而调用方（device 的 MultiCameraManager 取标定、
    // algorithm 的评分链取特征库）都假定"一个角色一台相机"。
    // 顺序依赖会表现为"同一份配置在不同机器上行为不同"这类
    // 极难复现的问题。
    const int n25  = static_cast<int>(std::count_if(
        channels_.begin(), channels_.end(),
        [](const data::CameraChannel& c) { return c.role == data::CameraRole::CAM25; }));
    const int n50  = static_cast<int>(std::count_if(
        channels_.begin(), channels_.end(),
        [](const data::CameraChannel& c) { return c.role == data::CameraRole::CAM50; }));
    const int n100 = static_cast<int>(std::count_if(
        channels_.begin(), channels_.end(),
        [](const data::CameraChannel& c) { return c.role == data::CameraRole::CAM100; }));

    if (n25 != 1 || n50 != 1 || n100 != 1)
    {
        channels_.clear();
        return false;
    }

    return true;
}

const data::CameraChannel* OpticalRig::getCamera(data::CameraRole role) const
{
    for (const data::CameraChannel& c : channels_)
    {
        if (c.role == role)
        {
            return &c;
        }
    }
    return nullptr;
}

const std::vector<data::CameraChannel>& OpticalRig::cameras() const
{
    return channels_;
}

data::OpticalRigCalibration OpticalRig::calibration() const
{
    return calibration_;
}

bool OpticalRig::setCalibration(const data::OpticalRigCalibration& calibration)
{
    if (channels_.empty())
    {
        // 通道表未建立：拒绝。静默接受会让 calibration_ 有值而
        // 通道内仍为空标定，于是 device 从通道读、CoordinateTransformer
        // 从 calibration() 读，两处拿到不同的结果。
        return false;
    }

    calibration_ = calibration;

    // 同步灌入各通道，保证"通道内标定"与"光机标定"始终一致
    // （见头文件对 setCalibration 的说明）。
    for (data::CameraChannel& c : channels_)
    {
        switch (c.role)
        {
        case data::CameraRole::CAM25:
            c.calibration = calibration_.cam25;
            break;
        case data::CameraRole::CAM50:
            c.calibration = calibration_.cam50;
            break;
        case data::CameraRole::CAM100:
            c.calibration = calibration_.cam100;
            break;
        }
    }

    return true;
}

}  // namespace optical
}  // namespace aircraft
