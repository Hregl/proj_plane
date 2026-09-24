#pragma once

// ============================================================================
//  src/algorithm/model/TargetModelManager.h
//
//  依据：SYS-12 §4（TargetModelManager：模型加载 / 特征库管理 / 版本管理 /
//                    焦段匹配）、§7（特征数据库目录）、§8（多焦段特征库）、
//        §13（模型版本管理）、§18（设计约束：CAD 与代码分离、特征库版本化、
//                    三焦段独立、支持模型替换）
//        ENG-02 §11.4（文件路径）、ENG-01 §10（model 子模块）、
//        ENG-10 §6（CAD 模型的输入要求）、ENG-09 §5.18 / §5.20 / §5.19
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 与 8.md §八 的巨大偏离：8.md 的版本是                              │
//  │     struct TargetModel { std::string modelId; };                    │
//  │     class TargetModelManager { bool load(path); TargetModel model(); };│
//  │ 即"一个只装了一个字符串的结构体"。它无法承载任何 PnP —— 没有三维点、  │
//  │ 没有描述子，2D-3D 对应无从建立。ENG-09 §5.20 已冻结                 │
//  │ `data::TargetModel { modelId; points3d; features; }`，本类使用该类型。│
//  │ 另外 SYS-12 §4.2 的接口是 `get(CameraRole)` 而非无参 `model()` ——    │
//  │ 三焦段的特征库物理分离（feature25/50/100.bin），必须按角色索引。      │
//  │ 裁决 C-09 已删除从未定义的 `FeatureDatabase` 容器。                   │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  目录布局（SYS-12 §7.3 / §13，冻结）：
//
//      models/aircraft/
//      ├── model.yaml        模型元数据（model_id / version / 生成信息）
//      ├── points3d.yaml     三维点表（B 类 CAD 结构点 + 纹理辅助点）
//      ├── feature25.bin     CAM25 的 A 类描述子库
//      ├── feature50.bin     CAM50 的 A 类描述子库
//      └── feature100.bin    CAM100 的 A 类描述子库
//
//  ⚠ 为什么 B 类（CAD 结构点）**没有** .bin：ENG-10 §2.6 冻结
//    "B 类的离线产物是 CAD 结构点坐标表（points3d.yaml），无需描述子"。
//    B 类在图像中的定位靠几何拟合（CadStructureLocator），不靠描述子匹配。
// ============================================================================

#include <string>

#include "data/CameraRole.h"
#include "data/TargetModel.h"

namespace aircraft
{
namespace algorithm
{

/// 目标模型管理（SYS-12 §4）。
///
/// 线程安全：**不保证**。模型在进程启动时一次性加载（ENG-10 §5.1 的注入
/// 时机"进程启动"），此后只读；SYS-09 §13 的并发模型里没有运行期重载。
/// 若将来需要热替换模型，须在 SYS-09 层面定义同步点。
class TargetModelManager
{
public:
    TargetModelManager() = default;

    /// 从模型目录加载全部文件（SYS-12 §4.2 的冻结方法名）。
    ///
    /// @param modelDir 形如 `models/aircraft` 的目录（不含尾斜杠亦可）。
    /// @return 是否加载成功。任一必需文件缺失或格式错误即失败，
    ///         **且不保留半加载状态**（失败后 `loaded()` 为 false、
    ///         `get()` 返回空模型）——半边加载的模型会让 PnP 用错误的点表
    ///         算出"看起来正常"的位姿，比直接失败危险得多。
    ///
    /// 必需文件：`model.yaml`、`points3d.yaml`、三个 `featureNN.bin`。
    /// 任一缺失即失败，理由同上：缺 feature50.bin 时 CAM50 通道会静默
    /// 退化为"没有 A 类特征"（匹配数恒为 0），而 SYS-12 §18 要求
    /// 三焦段各自优化，不允许一个通道静默降级。
    bool loadModel(const std::string& modelDir);

    /// 取指定焦段的模型（三焦段共用 points3d，各带自己的 A 类描述子库）。
    ///
    /// ⚠ 未加载或角色越界时返回**空模型**（`modelId` 为空、点表为空）。
    /// 调用方须以 `loaded()` 判定，不应靠"点表是否为空"推断 ——
    /// 一个合法的模型也可以只有 CAD 结构点而无纹理描述子。
    data::TargetModel get(data::CameraRole role) const;

    /// 是否已成功加载。
    bool loaded() const { return loaded_; }

    /// 当前模型的标识（`TargetModel::modelId`）。它同时是
    /// `runtime/match_stats.yaml` 的分组键（ENG-10 §4.1：换机型必须换表）。
    /// 未加载时返回空串。
    std::string modelId() const { return modelId_; }

    /// 模型版本号（SYS-12 §13）。写入 result.json 以支持可追溯（§18 约束 2）。
    std::string modelVersion() const { return modelVersion_; }

    /// 目标真实尺寸 L，单位 **m**（SYS-14 §10 的距离估计式需要它）。
    ///
    /// 取 `points3d` 包围盒的**最长边**。⚠ 前提是 points3d 覆盖目标的
    /// 全长方向（机头到尾翼）。若离线建模只放了局部结构点（例如只有翼尖），
    /// 本值会显著小于真实全长，进而使 `Z = f_x·L/l` 系统性偏小，
    /// 通道选择偏向长焦。ENG-10 §6 的 CAD 验收（抽检 6 个结构点）
    /// 正是防这一点，故本值在加载时会与 points3d 的包围盒一同校验：
    /// 任一边为零即加载失败。
    double targetRealSizeM() const { return targetRealSizeM_; }

    // ---- 离线工具 / 测试用 -------------------------------------------------

    /// 把一份模型写成 SYS-12 的目录布局（`model.yaml` / `points3d.yaml` /
    /// `featureNN.bin`）。
    ///
    /// 用途有两个，都不是运行期路径：
    ///   1. SYS-12 §12 的离线特征库生成工具（当前尚无独立工具，先由本入口
    ///      统一格式，避免"工具写的"与"运行期读的"两套格式）；
    ///   2. 测试夹具 —— 使 tests/algorithm 无需在仓库中携带二进制资产
    ///      （ENG-06 §10 的 golden 目录只放数据，不放生成器）。
    /// @param role 写哪一路特征库；三个角色各调一次，或由调用方循环。
    static bool saveFeatureLibrary(const std::string& path,
                                   data::CameraRole role,
                                   const data::TargetModel& model);

    /// 读回一路特征库（`loadModel` 内部使用，亦供测试单独校验格式）。
    static bool loadFeatureLibrary(const std::string& path,
                                   data::CameraRole role,
                                   data::TargetModel& inOut);

private:
    bool loaded_ = false;
    std::string modelId_;
    std::string modelVersion_;
    double targetRealSizeM_ = 0.0;

    /// 三个焦段各一份（三焦段特征库物理分离，SYS-12 §8）。
    /// 下标 = CameraRole 的序号（0/1/2）。
    data::TargetModel models_[3];
};

}  // namespace algorithm
}  // namespace aircraft
