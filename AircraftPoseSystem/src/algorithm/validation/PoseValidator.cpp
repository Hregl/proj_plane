// ============================================================================
//  src/algorithm/validation/PoseValidator.cpp
//
//  依据：SYS-07 §12.2（三项判据 + 姿态合理性）、ENG-09 §5.25 / §6.6
// ============================================================================

#include "algorithm/validation/PoseValidator.h"

#include <algorithm>
#include <cmath>

namespace aircraft
{
namespace algorithm
{

namespace
{

/// 把角度归一化到 (-180, 180]，单位 deg。
///
/// 必要性：PnP 输出的是旋转矩阵，欧拉角分解的结果可能落在 [0,360) 的任一
/// 分支上 —— 真值 -0.5° 与解出的 359.5° 是**同一个姿态**，但字面比较
/// `yaw ∈ [yawMin, yawMax]` 会把后者判为不合格。yawMin/yawMax 的用途是
/// "物理可达范围"（ENG-09 §6.6 文件头：例如 ±30°），一个合法但绕圈的
/// 表示被误判为超出范围，会让 VALIDATE 状态无谓重试直到 9001。
/// 该归一化是冻结文档未指定的一处选择，已记入 README §6。
double normalizeDeg(double deg)
{
    double d = std::fmod(deg, 360.0);
    if (d <= -180.0)
    {
        d += 360.0;
    }
    else if (d > 180.0)
    {
        d -= 360.0;
    }
    return d;
}

}  // namespace

PoseValidator::PoseValidator(const data::ValidationConfig& config)
    : config_(config)
{
}

double PoseValidator::confidence(double reprojectionError,
                                 double inlierRatio) const
{
    // ⚠ 本算式**没有任何冻结定义**（见头文件说明）。
    //
    // 取"最弱一环"（min）而不是加权平均，理由：
    //   · 两项判据是**合取**关系（都必须过），而合取的可信度天然由最弱项
    //     决定。加权平均会让"内点比例 0.99、重投影误差刚好在门槛上"这类
    //     组合得到一个漂亮的平均分，掩盖掉第二项本已临界的事实；
    //   · v1 没有可用的权重来源：`ValidationConfig` 里没有权重字段，
    //     凭空引入两个常数就是凭空引入两个可调参数（SYS-14 §20 约束 3）。
    //
    // 两个分项都归一化到 [0,1]：
    //   · 内点比例本身已在 [0,1]（ENG-09 §5.25）；
    //   · 重投影误差用"离门槛还有多远"表达：1 - err/maxReprojectionError。
    //     门槛值本身取 0 时（配置未注入的哨兵），该项无意义，退化为
    //     仅由内点比例决定 —— 这与 ENG-10 §5.3 "字段缺失 → 用默认值 +
    //     记日志"的处理一致，且不会因除零产生 inf。
    const double iRatio = std::isfinite(inlierRatio)
        ? std::min(1.0, std::max(0.0, inlierRatio))
        : 0.0;

    double errScore = 1.0;
    if (config_.maxReprojectionError > 0.0 && std::isfinite(reprojectionError))
    {
        errScore = 1.0 - reprojectionError / config_.maxReprojectionError;
        errScore = std::min(1.0, std::max(0.0, errScore));
    }
    else if (!std::isfinite(reprojectionError))
    {
        errScore = 0.0;
    }

    return std::min(iRatio, errScore);
}

bool PoseValidator::validate(const data::ShipPoseResult& result,
                             double inlierRatio,
                             int matchCount,
                             data::PoseValidationResult& out) const
{
    // 三个字段**先填后判**：SYS-08 §7.5 的失败记录需要它们
    // （"哪一项没过、值是多少"），提前 return 会让 result.json 里
    // 只剩一个 valid=false。
    out.reprojectionError = result.reprojectionError;
    out.inlierRatio = std::min(1.0, std::max(0.0,
        std::isfinite(inlierRatio) ? inlierRatio : 0.0));
    out.confidence = confidence(result.reprojectionError, out.inlierRatio);
    out.valid = false;
    out.reason = data::PoseValidationReason::OK;

    // `matchCount` 不单独设门槛（门槛是内点比例），但对"内点比例为 0"
    // 这一情形，它区分了两种完全不同的原因：根本没有对应（0 个）与
    // 有对应但全被判为外点。记录到日志里以保留这个区分。
    (void)matchCount;

    // ① 解算本身必须成功。success=false 时其余字段无意义
    //    （ENG-09 §5.24："false 时其余字段无意义"）。
    if (!result.success)
    {
        return false;
    }

    // ②-a 非有限值闸门（裁决 C-008）。
    //
    // ⚠ 位置：**在 ① 之后、全部阈值判据之前**。
    //
    //   在阈值判据之前 —— 三个实测理由，缺一这条闸门就会被绕开：
    //     · ⑤ 的 yaw 区间判据默认**根本不生效**（`yawMin == yawMax == 0`
    //       是"未配置"哨兵，见 ⑤ 的说明），此时 `yaw = NaN` 连比较都不做；
    //     · 即便配置了区间，`NaN < min || NaN > max` **恒为 false**，
    //       NaN 会以"通过"告终；而 `pitch` / `roll` 连判据都没有；
    //     · ② 只守 `reprojectionError` 一个量，其余姿态量（yaw/pitch/roll
    //       与 `aircraftToShip` 的 12 个元素）在整个 validate() 里
    //       **没有任何检查**。
    //
    //   但在 ① 之后 —— `success == false` 时这些字段按冻结文档**本无意义**，
    //   对无意义的字段做有限性检查，会把"解算失败"这个真实原因覆盖成
    //   一个由垃圾值引发的误报（`NON_FINITE_VALUE`），而处置方向完全不同：
    //   前者是"算法没解出来"，后者是"算出来一个不是数的东西"。
    //
    // 可达性不是假想的：此前的实测路径是"yaw = NaN 的姿态一路走到 SAVE 并
    // 落盘"，落盘侧只能靠 `non_finite_fields` 点名表兜住（`num()` 把 NaN
    // 写成 0）—— 那是**最后一道**防线，而这里应当是第一道。
    //
    // 先例：`CoordinateTransformer.cpp` 的 `allFinite()` 已对旋转矩阵做过
    // 同样的检查（用例 `RejectsNonFiniteInput`）。处置方向也一致：
    // **拒绝**，而不是"当作 0 继续算"。
    //
    // ⚠ 判据覆盖 `ShipPoseResult` 的**全部**浮点量，不多不少：
    //   12 个变换元素 + 3 个欧拉角 + 重投影误差。`success` 是 bool；
    //   非法位置参量（inlierRatio）已在上面被夹到 [0,1]。
    {
        const data::Transform& tf = result.aircraftToShip;
        bool finite = true;
        for (int r = 0; r < 3 && finite; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                if (!std::isfinite(tf.rotation(r, c)))
                {
                    finite = false;
                    break;
                }
            }
        }
        for (int i = 0; i < 3 && finite; ++i)
        {
            if (!std::isfinite(tf.translation[i]))
            {
                finite = false;
            }
        }
        if (finite
            && (!std::isfinite(result.yaw) || !std::isfinite(result.pitch)
                || !std::isfinite(result.roll)
                || !std::isfinite(result.reprojectionError)))
        {
            finite = false;
        }

        if (!finite)
        {
            // reason 指向"上游算法"，而不是任何阈值：下面的三个数值字段
            // 此时**全部不可信**（可能本身就是 NaN，或在 NaN 参与下算出），
            // 把它们当作"差多少"来读会把现场引向调阈值。
            out.reason = data::PoseValidationReason::NON_FINITE_VALUE;
            out.valid  = false;
            return false;
        }
    }

    // ② 重投影误差（SYS-07 §12.2 第一项）。
    //
    //    ⚠ 此处原来另有一道 `!std::isfinite(reprojectionError)` 检查。
    //    裁决 C-008 起它由上面的闸门统一承担（闸门检查的**同一个字段**，
    //    且更早执行），故在此删除 —— 留着就是同一件事的第二份判据，
    //    两处迟早不一致（改了一处忘了另一处，而两者都"看起来正常"）。
    //    `err > maxErr` 的写法对 NaN 恒为 false 这一性质已不再需要单独兜底。
    if (config_.maxReprojectionError > 0.0
        && result.reprojectionError > config_.maxReprojectionError)
    {
        return false;
    }

    // ③ 内点比例（第二项）。下限为 0（未配置哨兵）时不设限。
    if (config_.minInlierRatio > 0.0
        && out.inlierRatio < config_.minInlierRatio)
    {
        return false;
    }

    // ④ 置信度（第三项）。
    if (config_.minConfidence > 0.0 && out.confidence < config_.minConfidence)
    {
        return false;
    }

    // ⑤ 姿态合理性（SYS-07 §12.2：Yaw 范围）。
    //    ⚠ 判据只在 `yawMin < yawMax` 时生效：`ValidationConfig` 的默认值
    //    是 yawMin = yawMax = 0（ENG-09 §6.6），这是一个**未配置哨兵**。
    //    若照字面执行"yaw ∈ [0,0]"，任何非零 Yaw 都会被判不合格，
    //    而失败信息只会表现为 VALIDATE 反复重试 —— 与"配置没填"这个真实
    //    原因毫无关联（ENG-10 §5.3 对缺字段的处理是"默认值 + 记日志"，
    //    不是"用一个空区间把所有结果拒掉"）。
    //    yawMin > yawMax 是配置错误（ENG-10 §5.3：越界 → 启动失败），
    //    在配置加载阶段拦截，此处按"未配置"处理。
    if (config_.yawMin < config_.yawMax)
    {
        const double yaw = normalizeDeg(result.yaw);
        if (yaw < config_.yawMin || yaw > config_.yawMax)
        {
            return false;
        }
    }

    out.valid = true;
    return true;
}

}  // namespace algorithm
}  // namespace aircraft
