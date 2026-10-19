#pragma once

// ============================================================================
//  src/infrastructure/recorder/Recorder.h
//
//  依据：ENG-01 §11（infrastructure/recorder）、ENG-03 §12.7（libinfrastructure
//        含 Recorder）、SYS-09 §12（RecorderWorker 的产出）、
//        SYS-04 §6.4（IF-FILE-04 测量结果包的字段清单）、
//        SYS-04 IF-THR-05（记录线程接口）
//
//  ⚠ **本类不实现 `application::IRecorderSink`**，由 app 侧适配器桥接。
//    原因是冻结文档之间的一处反向依赖：
//      · `IRecorderSink` 定义在 `application/MeasurementController.h`；
//      · 本类的归属被 ENG-03 §12.7 冻结在 `infrastructure`。
//    而 ENG-01 §17 的依赖方向是 `application → infrastructure`，
//    故 infrastructure 引用 application 是**反向依赖**（§18 禁止），
//    且构建期就会失败：libinfrastructure 的依赖表里没有 application
//    的包含路径（`aps_add_module_library(infrastructure DEPENDS data)`）。
//
//    这与裁决 C-21 处理 `IMatchStatsStore` 的情形同类（跨层接口的落层
//    问题），但结论不同：`IMatchStatsStore` 的接口可以下沉到 data/，
//    而 `IRecorderSink` 已由 SYS-04 冻结为 `MeasurementController` 的
//    一部分（改它的位置等于改冻结接口）。因此此处采用**适配器**：
//    本类只依赖 data/，由 app（全工程唯一的装配点，ENG-01 §14）
//    用一个 10 行的 adapter 把它接到 `IRecorderSink` 上。
//    已登记待裁决：是否把 `IRecorderSink` 也下沉到 data/。
//
//  ⚠ 结果包的完整性（SYS-04 §6.4）当前**只部分满足**：
//    `result.json` 的必含字段里，以下字段在冻结的 `data::MeasurementTask`
//    中根本没有承载者（该结构体只有 taskId / state / result / validation
//    四个成员）：
//        camera_used, selected_camera, score, quality,
//        feature_count, match_count
//    它们分别属于 `MeasurementSelectionResult` / `ImageQuality`，
//    而 `IRecorderSink::save()` 只接收 `MeasurementTask`。
//    本类**不伪造**这些字段，而是把它们列进 result.json 的
//    `missing_required_fields` 数组 —— 让"结果包不完整"成为
//    机器可读的事实，而不是一个看起来正常的 json。
//    同理，SYS-09 §12 要求的 cam25|50|100.raw（需要图像）、
//    turntable.json（需要转台角度）、log.txt（需要按任务切分日志）
//    当前也写不出，且原因相同：`save(task)` 的入参不够。
//    已登记待裁决：是否把 `IRecorderSink::save` 的入参扩展为
//    一个 RecordingContext（图像 + 选择结果 + 转台状态 + 日志缓冲）。
//
//    以上"只部分满足"的清单已由裁决 C-002（改入参为 MeasurementRecord）与
//    裁决 C-007（失败任务同样落盘 + failure.json）大幅收窄：
//    现在仍进 `missing_required_fields` 的只剩 model_type（C-003 的
//    synthetic / production 机型库未实施时为空）。
//    见 .cpp 的 writeResultJson 说明。
// ============================================================================

#include <string>
#include <vector>

#include "data/MeasurementRecord.h"
#include "data/MeasurementTask.h"
#include "data/MultiCameraFrame.h"
#include "data/SystemConfig.h"
#include "data/TurntableState.h"

namespace aircraft
{
namespace infrastructure
{

/// 测量结果包 `measurement_xxx/` 的落盘者。
class Recorder
{
public:
    /// @param system        取 `outputDir`（结果包建在其下）。
    /// @param configDir     7 个 yaml 所在目录，用于写 config_snapshot/
    ///                      （SYS-04 §6.4 冻结：结果包必须包含本次生效的
    ///                      全部配置，否则回放时的判定阈值无从复现）。
    /// @param calibrationId 标定版本（结果包可直接追溯）。
    /// @param modelId       目标模型标识。
    /// @param featureVersion 特征库版本。
    Recorder(const data::SystemConfig& system,
             std::string configDir,
             std::string calibrationId,
             std::string modelId,
             std::string featureVersion);

    /// 保存一个测量结果包。**只在任务终态（COMPLETE / FAILED）时调用** ——
    /// 与 `IMatchStatsStore` 的写回时机同源（ENG-10 §4.4）。
    ///
    /// R11 保存事务（ENG-10 §4.5）：
    ///   · 全部文件先写入 `outputDir` 下**本次唯一**的暂存目录
    ///     （`.aps_tmp_*`，与最终目录同一文件系统）；
    ///   · 全部写成功后才用 `renameat2(RENAME_NOREPLACE)` 一次发布为
    ///     `outputDir/<taskId>/`。读者只会看到"没有包"或"完整包"，
    ///     不会看到半包；
    ///   · 最终目录**已存在时保存失败**，不覆盖、不改写已有结果；
    ///   · 任一步失败 ⇒ 删除暂存目录，最终目录不出现。
    ///
    /// ⚠ 该事务保证的是**发布的原子性与不覆盖**，**不是断电持久性**：
    ///   本实现不 fsync 文件数据与父目录。save() 返回 true 只表示包已对
    ///   读者可见，掉电后是否仍在是另一件未验证的事。
    ///
    /// @return 结果目录、result.json、failure.json 与 config_snapshot/
    ///         全部写入并**成功发布**时返回 true；任一失败返回 false，
    ///         原因见 lastErrorText()。返回 false **不抛异常**：落盘失败
    ///         不应让 UI 线程崩溃，但必须让调用方把错误写进日志与界面。
    ///
    /// ⚠ **失败任务也应当调用本方法**（裁决 C-007）。此前失败任务不落盘，
    ///   代价是现场只留一行日志，而"过程走到哪一步"永远丢失。失败包与成功包
    ///   的差别只有一处：`record.bestFrame` 为空，故 `writeRawFrames` 不写
    ///   任何 `cam*.raw`（省空间），其余文件齐备。
    /// ⚠ 入参由 `data::MeasurementTask`（4 字段）改为
    /// `data::MeasurementRecord`（裁决 C-002，2026-09-23 批准）。
    /// 这一改动**正是本文件头"结果包的完整性当前只部分满足"那一段的结论**：
    /// 旧入参下 camera_used / selected_camera / score / quality /
    /// feature_count / match_count 六个必含字段无承载者，cam25|50|100.raw
    /// 与 turntable.json 也写不出，只能把它们列进
    /// `missing_required_fields`。
    bool save(const data::MeasurementRecord& record);

    /// 最近一次 save **成功发布**的结果目录。失败或无调用时为空。
    ///
    /// ⚠ 失败时为空是 R11 事务的直接结果：最终目录从未出现，因此不存在
    ///   "失败时已创建的部分目录"可返回。需要查看失败原因用 lastErrorText()。
    const std::string& lastPackageDir() const { return lastPackageDir_; }

    /// 最近一次失败的原因；成功时为空。
    const std::string& lastErrorText() const { return lastErrorText_; }

    /// 最近一次 save 中"因数据源缺失而未能写入"的必含字段名
    /// （SYS-04 §6.4 的清单，见文件头说明）。
    const std::vector<std::string>& lastMissingFields() const
    {
        return lastMissingFields_;
    }

    /// 结果包中 result.json 的完整路径（lastPackageDir + "/result.json"）。
    std::string lastResultPath() const;

    /// 构造时注入的标定版本 / 模型标识。
    ///
    /// ⚠ 存在的理由不是"顺手读一下"，而是**避免同一件事有两个解析点**：
    /// `RecorderSinkAdapter`（app 侧）需要把标定版本与模型标识补进
    /// `MeasurementRecord`（它们不是测量过程的产物，见
    /// `MeasurementController::buildRecord()` 的说明）。若适配器自己再去
    /// 读一遍 `ctx_.calibration` / `ctx_.models` 并再写一遍
    /// `empty() ? "unknown" : ...` 的兜底，就会产生**第二份**同名逻辑 ——
    /// 与本文件/SystemInitializer 里已记录的"多份副本"是同一类问题
    /// （两处迟早不一致，且不一致时无人察觉）。故由本类持有并提供读取。
    const std::string& calibrationId() const { return calibrationId_; }
    const std::string& modelId() const { return modelId_; }

private:
    /// 写 cam25|50|100.raw。
    ///
    /// ⚠ 文件里的字节**不一定是裸像素缓冲**，取决于该路帧携带的字段
    ///   （011-A1 §2.1 第 7 条的三分支）：有原始载荷 ⇒ 写原始载荷；
    ///   `RawOptional` 且 8 位而载荷缺失 ⇒ 写 8U 显示图并**如实标注**
    ///   `data_source = image`；`RawRequired` 或 12 位而载荷缺失 ⇒
    ///   **报契约错误、不写该文件**（禁止静默回落成 8 位图）。
    ///   故解码依据是 result.json 里该路的 `data_source` 与
    ///   `pixel_format`／`valid_bits`／`packing`／`declared_byte_order`
    ///   —— **不是** `display_image` 的 `width`/`height`/`type`
    ///   （那三项描述的是显示图）。见 .cpp 的格式选择说明。
    bool writeRawFrames(const data::MultiCameraFrame& frame,
                        const std::string& packageDir,
                        std::string& error);

    /// 写 turntable.json（SYS-09 §12）。与 result.json 的 turntable 块同源。
    bool writeTurntableJson(const data::TurntableState& turntable,
                            const std::string& packageDir,
                            std::string& error);

    bool writeResultJson(const data::MeasurementRecord& record,
                         const std::string& packageDir,
                         std::string& error);

    /// 写 failure.json：失败任务的根因、终点与**全量**迁移轨迹（裁决 C-007）。
    ///
    /// ⚠ 与 result.json 里的 `failure` 块是同一份 `record.failure` 的
    ///   两种粒度：result.json 留摘要（首/末状态与错误），本文件留全量
    ///   `history`。**两个文件都写**，即使任务成功 —— 成功任务在这里得到的是
    ///   "走过哪些状态、全程无错误"，而不是一个缺失的文件（缺文件与
    ///   "旧版没有这个功能"在目录列表里无法区分）。
    bool writeFailureJson(const data::MeasurementRecord& record,
                          const std::string& packageDir,
                          std::string& error);
    bool writeConfigSnapshot(const std::string& packageDir, std::string& error);
    bool writeCalibrationId(const std::string& packageDir, std::string& error);

    std::string outputDir_;
    std::string configDir_;
    std::string calibrationId_;
    std::string modelId_;
    std::string featureVersion_;

    std::string              lastPackageDir_;
    std::string              lastErrorText_;
    std::vector<std::string> lastMissingFields_;
};

}  // namespace infrastructure
}  // namespace aircraft
