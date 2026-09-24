#pragma once

// ============================================================================
//  src/data/MeasurementRecord.h
//
//  依据：裁决 C-002（V2.1-C01_架构裁决变更说明.md §C-002，2026-09-23 已批准）
//        裁决 C-007（§C-007：失败也要留档）
//        H-002（冻结：标定版本 / 模型标识 / 软件版本必须进入记录）
//        SYS-09 §12（result.json 由 Recorder 产出）
//        ENG-08 §16（版本基线可见性）
//
//  作用：**一次测量的完整记录**，是 SAVE 状态落盘的对象，也是离线追溯与回放的
//  唯一入口。`measurement_xxx/result.json` 即本结构体的序列化
//  （与 MeasurementTask 同源：本结构体**包含**它，而非取代它）。
//
//  ⚠ 与 MeasurementTask 的关系（不要合并，也不要二选一）：
//  MeasurementTask（ENG-09 §5.26，冻结）表达"这一**任务**是什么、结果如何"，
//  只有 4 个字段且**不得增改**（README §6 第 19 行）。本结构体表达
//  "这次测量的**证据链**"——用哪一帧解算、质量如何、失败在哪一步、依据哪份
//  标定与哪个模型。二者是"结论"与"依据"的关系：只有结论的结果包无法被质疑，
//  也无法被复现。
//
//  ⚠ 为什么放 data 而不是 application（与裁决 C-13 同一条理由）：
//  本结构体由 SAVE 状态写入、由 infrastructure 层的 RecorderWorker 序列化。
//  若定义在 application，infrastructure 就必须反向依赖 application ——
//  违反 ENG-01 §18。跨层的类型必须落在依赖关系的**最低公共层**（ENG-10 §5.1）。
//
//  ⚠ 内存语义（不是细节，会真的吃内存）：
//  bestFrame 内含 3 个 cv::Mat（浅拷贝、引用计数）。本结构体一旦持有它，
//  就**延长了这三帧图像的生命周期**——2448×2048 灰度约 5 MB/路，三路约 15 MB。
//  故要求：SAVE 完成或任务终止后必须释放（MeasurementController 负责），
//  否则连续测量会逐次累积（见 V2.1-C02_实施设计说明.md §2.6）。
//  这也是 FailureTrace 刻意**不含原图**的原因（裁决 C-007）：失败任务的记录
//  可能长期留存，不能按帧计内存。
// ============================================================================

#include <string>

#include "data/CameraRole.h"
#include "data/FailureTrace.h"
#include "data/ImageQuality.h"
#include "data/MeasurementStatistics.h"
#include "data/MeasurementTask.h"
#include "data/MultiCameraFrame.h"
#include "data/TurntableState.h"

namespace aircraft
{
namespace data
{

/// 一次测量的完整记录（裁决 C-002 + C-007 + H-002）。
struct MeasurementRecord
{
    /// 任务标识、终态与姿态结论（ENG-09 §5.26，冻结类型）。
    MeasurementTask task;

    /// **产出该姿态的那一次**采集（三路齐，同一次曝光）。
    ///
    /// ⚠ 这里**必须**是"被解算的那一帧"，而不是"最后一次采集"：
    /// 二者不同帧时，记录里的一切（质量、统计）都会与被解算的图像对不上，
    /// 且没有任何报错。C-02 修复的正是这一点
    /// （见 V2.1-C02_实施设计说明.md §1.1 D-C02-1）。
    MultiCameraFrame bestFrame;

    /// 匹配环节统计（见 MeasurementStatistics 的字段说明）。
    MeasurementStatistics statistics;

    /// 采集/解算时转台的位姿。用于事后判断"这个姿态是在转台什么位置上测的"
    /// （转台角度是粗姿态来源，也是误差归因的起点）。
    TurntableState turntable;

    /// 本次测量使用的焦段（MEASURE_SELECT 的结论）。
    CameraRole selectedCamera = CameraRole::CAM25;

    /// 该焦段的加权总分（MeasurementSelectionResult::score）。
    /// 只有被选中的角色而无分数时，无法回答"它赢了多少"——
    /// 而"险胜"与"压倒性胜出"对通道选择的可信度判断完全不同。
    double selectedScore = 0.0;

    /// 被选中那一次采集的图像质量（ImageQuality）。
    ImageQuality selectedQuality;

    /// 失败根因与轨迹（裁决 C-007）。成功任务时 firstError.code == 0。
    FailureTrace failure;

    /// 本次测量是否在**降级状态**下完成（相机不足 3 路 / 触发降为软触发）。
    ///
    /// ⚠ 为什么它是"测量事实"而不是"控制器状态"：
    /// 曾经的理由是不加这两个字段、由 Recorder 落盘时去问控制器
    /// （README §6 第 19 行）。该理由不成立 —— 控制器持有的是**瞬态**：
    /// 任务结束后 `degraded()` 反映的是"此刻"，而不是"这次测量当时"。
    /// 若任务结束到落盘之间发生过任何一次降级变化，写进包里的就是错的值，
    /// 且**没有任何报错**。这两个量属于"本次测量的既成事实"，
    /// 与 `taskId` 同级，必须在测量结束时**取值冻结**。
    bool degraded = false;

    /// 本次测量实际可用的相机路数（SYS-08 §7.5 的 `cameras_available`）。
    /// 与 `degraded` 同源、同时刻取值，理由同上。
    int camerasAvailable = 0;

    /// 本次测量所依据的标定标识（H-002）。
    /// 标定会被更新，而姿态结论依赖当时的标定；没有标识就无法判断
    /// "这份历史结果用的是哪版标定"。
    std::string calibrationId;

    /// 本次测量所使用的目标模型标识（H-002）。
    std::string modelId;

    /// 该模型的**类型**（`synthetic` / `production`，C-003 的两种机型库）。
    ///
    /// ⚠ 为什么 `modelId` 不够：`modelId` 只说"用了哪个模型"，不说
    /// "它是不是合成模型"。而合成模型产出的姿态**不具测量意义**（C-003），
    /// 事后追溯时必须一眼看出"这份结果不能用"，而不是再去翻模型文件。
    /// 只留 `modelId` 会让"结果是否可信"这个判断依赖包外的状态。
    std::string modelType;

    /// 产出该结果的软件版本（H-002 + ENG-08 §16）。
    /// 没有它，历史结果无法与"当时的算法行为"对应——
    /// 而算法会改，结果数值也会随之改变。
    std::string softwareVersion;
};

}  // namespace data
}  // namespace aircraft
