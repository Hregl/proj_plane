#pragma once

// ============================================================================
//  src/device/camera/IMultiCameraManager.h
//
//  依据：SYS-06 §5.1 / §5.2（接口冻结）、ENG-09 §4.3、ENG-01 §6
//
//  作用：三相机同步采集的管理者。它把三个 backend 的并发采集
//  收敛为**一个** MultiCameraFrame，使上层（application / algorithm）
//  永远只面对"三个视角属于同一时刻"这一个前提，而不必各自处理
//  三路异步到达的图像。
//
//  ⚠ 接口签名**逐字**遵循 SYS-06 §5.2（4 个纯虚函数），未增删。
//  文件位置与命名空间的偏离同 ICameraBackend.h。
//
//  ⚠ 本接口**不承载降级信息**，这是有意的：
//  SYS-08 §7.5 要求结果包记录 `degraded=true` 与 `cameras_available`，
//  而这两个事实属于**管理器状态**，不属于某一帧。
//  若把它们塞进 MultiCameraFrame，则每帧都要重复携带同一个管理器级事实，
//  且存在"帧说降级了、管理器说没降级"的不一致窗口。
//  故：帧只含数据，状态由具象类 MultiCameraManager 的查询方法提供
//  （接口本身保持冻结，不因此增补方法）。
// ============================================================================

#include "data/ErrorInfo.h"
#include "data/MultiCameraFrame.h"

namespace aircraft
{
namespace device
{

/// 多相机管理器接口（SYS-06 §5.2）。
///
/// ⚠ 降级语义（SYS-08 §7.5，实现必须遵守）：
///   | 可用相机数 | 行为 |
///   |    3      | 正常 |
///   |    2      | **降级继续**，禁用故障相机 |
///   |   ≤1      | 直接失败，ErrorInfo{code=1001} |
///
/// 因此 capture() 在只有 2 台可用时**必须返回 true**（而不是失败）——
/// 把"少一台"当作失败会白白丢弃 SYS-08 §7.5 明确允许的降级能力，
/// 让一次本可完成的测量变成 FAILED。
class IMultiCameraManager
{
public:
    virtual ~IMultiCameraManager() = default;

    /// 初始化全部相机。返回 false 表示可用相机数不足（<2），
    /// 上层应直接 FAILED 并置 kErrCameraInsufficient(1001)。
    virtual bool initializeAll() = 0;

    /// 启动全部相机的采集。返回 false 的含义同 initializeAll()。
    virtual bool startAll() = 0;

    /// 停止全部相机。允许重复调用。
    virtual void stopAll() = 0;

    /// 同步采集一帧多相机图像。
    ///
    /// 成功时 frame.triggerTimestamp 取各相机 timestampNs 的**最大值**
    /// （ENG-09 §2.5 冻结）。取最大而非最小或均值的原因：
    /// 它保证 triggerTimestamp 不早于任何一路的实际曝光时刻，
    /// 使"该帧代表的时刻"这一语义不会超前于尚未采到的数据。
    ///
    /// 不可用的相机会在 frame 中留下默认构造的 ImageFrame
    /// （image 为空）。调用方应以 `frame.camXX.image.empty()` 判断该路是否
    /// 有效 —— 这比引入哨兵枚举可靠，见 CameraRole.h 的说明。
    ///
    /// @return 可用相机数 <2 时返回 false（上层应置 1001 并 FAILED）。
    virtual bool capture(data::MultiCameraFrame& frame) = 0;

    /// 上一次操作失败的原因（裁决 C-006）。
    ///
    /// ⚠ 为什么必须由接口提供而不是由上层猜：`capture()` 只回一个 bool，
    /// 而它失败的原因至少有两类**处置完全不同**的情形 ——
    /// "可用相机数不足"（1001，硬件故障，不重试）与
    /// "三路时间戳超差"（3002，瞬态，重采即可）。
    /// 上层若只凭 false 就自己编一个码，等于把这两类混为一谈。
    ///
    /// 实现者**应当**填写本字段；这里给出默认实现是为了不破坏既有实现
    /// （本接口不是冻结 ICD，但也没必要为此让所有实现者改代码）。
    /// 未填写（code == 0）时上层按兜底码 9004 记录，而**不会**冒充某个
    /// 具体码 —— 宁可少报一个具体码，也不要凭空造一个不成立的根因。
    virtual data::ErrorInfo lastError() const { return data::ErrorInfo{}; }
};

}  // namespace device
}  // namespace aircraft
