// ============================================================================
//  src/algorithm/model/TargetModelManager.cpp
//
//  依据：SYS-12 §4 / §7 / §8 / §13 / §18、ENG-10 §6、ENG-09 §5.18~§5.20
//
//  文件格式说明见本文件 §格式 一节（SYS-12 §12 只规定"离线生成特征库"，
//  未规定二进制布局，故格式在此定义并由 saveFeatureLibrary 统一产出，
//  避免"离线工具写的"与"运行期读的"两套格式并存）。
// ============================================================================

#include "algorithm/model/TargetModelManager.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace aircraft
{
namespace algorithm
{

namespace
{

// ---------------------------------------------------------------------------
//  §格式：featureNN.bin（本文件定义）
// ---------------------------------------------------------------------------
//
//  偏移  类型        含义
//  0     char[4]     魔数 "APSF"（AircraftPoseSystem Feature）
//  4     uint32      格式版本，当前 1
//  8     uint32      role 序号（0=CAM25, 1=CAM50, 2=CAM100）
//  12    uint32      N（描述子条数）
//  16    uint32      D（每条描述子的维度）
//  20    uint32      元素类型，当前 4 = CV_32F
//  24    int32[N]    每条描述子关联的三维点索引（FeatureDescriptor::point3dIndex）
//  24+4N float32[N*D] 描述子数据，行优先
//
//  选用小端、定长布局的理由：离线工具（SYS-12 §12）与运行期都可能在不同
//  架构上编译，而 OpenCV 的 FileStorage 不擅长存大块浮点矩阵；
//  定长头 + 连续数据块可用一次 fread 读入，与 cv::Mat 的行优先布局一致。
//  ⚠ 未做端序转换：本工程的目标平台（UOS x86_64）与离线工具同为小端。
//    若将来引入大端平台，必须在此加端序判定 —— 否则描述子会整体错位，
//    而**不会报错**（浮点乱码仍能通过匹配，只是匹配结果无意义）。
//
//  ⚠ 格式版本与 role 都写入头部并在读取时校验：一个 CAM50 的库被误命名为
//    feature25.bin 是很容易发生的操作失误（复制粘贴改错一个数字），
//    只靠文件名无法发现，而后果是 CAM25 通道用 CAM50 的描述子匹配，
//    匹配数极低但不为零，最终表现为"25mm 通道图像质量差"这一错误结论。

constexpr char     kMagic[4]      = {'A', 'P', 'S', 'F'};
constexpr uint32_t kFormatVersion = 1;
constexpr uint32_t kElemFloat32   = 4;

/// CameraRole → 序号（0/1/2，与 CameraRole.h 的枚举顺序一致）。
///
/// ⚠ 不用 `static_cast<int>(role)` 直接当序号：CameraRole 的枚举顺序
/// 若被调整（例如按焦段长度重排），序号会整体错位，而 .bin 头部记录的
/// 是**旧序号**，校验就会通过 —— 两个通道的描述子互换且无人发现。
/// 显式映射把"顺序"这一隐含契约变成一处唯一可改的代码。
uint32_t roleIndex(data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM25:  return 0;
    case data::CameraRole::CAM50:  return 1;
    case data::CameraRole::CAM100: return 2;
    }
    return 0;
}

bool roleFromIndex(uint32_t idx, data::CameraRole& out)
{
    switch (idx)
    {
    case 0: out = data::CameraRole::CAM25;  return true;
    case 1: out = data::CameraRole::CAM50;  return true;
    case 2: out = data::CameraRole::CAM100; return true;
    default: return false;
    }
}

/// 拼接目录与文件名，容忍 modelDir 带或不带尾斜杠。
std::string joinPath(const std::string& dir, const std::string& name)
{
    if (dir.empty())
    {
        return name;
    }
    if (dir.back() == '/')
    {
        return dir + name;
    }
    return dir + "/" + name;
}

const char* featureFileName(data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM25:  return "feature25.bin";
    case data::CameraRole::CAM50:  return "feature50.bin";
    case data::CameraRole::CAM100: return "feature100.bin";
    }
    return "feature.bin";
}

/// 读 points3d.yaml（SYS-12 §5.2 / §7.3）。
///
/// 用 `cv::FileStorage` 而非自写 YAML 解析：OpenCV 已在依赖内
/// （ENG-03 §12.5 给 algorithm 的依赖就是 data + OpenCV），
/// 而手写 YAML 解析器是这类工程里最典型的"自己造轮子然后漏掉转义/引号"
/// 的缺陷来源。
///
/// 期望的文档结构（SYS-12 §5.2 的三维点数据）：
///
///     model_id: aircraft_model_v1
///     version: 1
///     points:
///       - { id: 1, x: -5.0, y: 0.0, z: 0.0, type: cad }
///       - { id: 2, x:  5.0, y: 0.0, z: 0.0, type: cad }
bool loadPoints3d(const std::string& path,
                  std::string& modelId,
                  std::string& version,
                  std::vector<data::ModelPoint3D>& points)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        return false;
    }

    fs["model_id"] >> modelId;
    // version 在 YAML 里可能是整数，也可能是字符串："1" 与 1 都要能读。
    const cv::FileNode v = fs["version"];
    if (v.isString())
    {
        version = static_cast<std::string>(v);
    }
    else if (v.isInt())
    {
        version = std::to_string(static_cast<int>(v));
    }

    const cv::FileNode pts = fs["points"];
    if (pts.empty() || !pts.isSeq())
    {
        return false;
    }

    points.clear();
    for (const cv::FileNode& n : pts)
    {
        data::ModelPoint3D p;
        n["id"] >> p.id;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        n["x"] >> x;
        n["y"] >> y;
        n["z"] >> z;
        n["type"] >> p.featureType;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        {
            return false;
        }
        // 单位：yaml 中即为 **米**（ENG-09 §2.3 冻结；SYS-11 的标定工具链
        // 默认输出毫米，转换必须在 CalibrationManager::load() 内一次完成，
        // 不得泄漏到算法层 —— 故此处不做任何单位换算，读到什么就是什么）。
        p.position = cv::Point3f(static_cast<float>(x),
                                 static_cast<float>(y),
                                 static_cast<float>(z));
        points.push_back(p);
    }

    return !points.empty();
}

/// 读 model.yaml（SYS-12 §7 的目录清单 + §13 的版本管理 + SYS-04 §6.3
/// "model.yaml 必须记录：模型来源、生成时间、特征算法版本、参数"）。
///
/// 期望结构：
///
///     model_id: aircraft_model_v1
///     version: 1
///     model_source: cad/aircraft_v3.step
///     generated_at: 2026-05-01T10:00:00
///     feature_algorithm_version: sift-4.6.0
///
/// ⚠ 为什么本文件是**必需**的，即使 `points3d.yaml` 里也有 model_id：
/// `model_id`/`version` 是 `runtime/match_stats.yaml` 的分组键
/// （ENG-10 §4.1：换机型必须换统计表）与 result.json 的可追溯字段
/// （SYS-12 §18 约束 2）。SYS-04 §6.3 把这些元数据归给 model.yaml，
/// 而 points3d.yaml 的职责是**点表**。两者都写一份时，model.yaml 是
/// 权威来源（本节优先），points3d.yaml 里的值只在 model.yaml 未给出时兜底 ——
/// 否则同一份模型会有两个可独立编辑的身份来源，换涂装时只改一处即可让
/// 统计表与模型版本对不上，而两边都"看起来正常"。
///
/// @param modelId/version 输出（可能被本函数覆盖为 model.yaml 的值）。
/// @return model.yaml 是否存在且可解析且含非空 model_id。
bool loadModelMeta(const std::string& path,
                   std::string& modelId,
                   std::string& version)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        return false;
    }

    std::string metaId;
    fs["model_id"] >> metaId;
    if (metaId.empty())
    {
        return false;
    }

    const cv::FileNode v = fs["version"];
    std::string metaVersion;
    if (v.isString())
    {
        metaVersion = static_cast<std::string>(v);
    }
    else if (v.isInt())
    {
        metaVersion = std::to_string(static_cast<int>(v));
    }

    modelId = metaId;
    if (!metaVersion.empty())
    {
        version = metaVersion;
    }
    return true;
}

}  // namespace

// ---------------------------------------------------------------------------

bool TargetModelManager::saveFeatureLibrary(const std::string& path,
                                            data::CameraRole role,
                                            const data::TargetModel& model)
{
    // 描述子维度以第一条为准；空库允许 N=0、D=0（合法的"尚无纹理特征"状态）。
    const uint32_t n = static_cast<uint32_t>(model.features.size());
    uint32_t d = 0;
    if (n > 0 && !model.features.front().descriptor.empty())
    {
        d = static_cast<uint32_t>(model.features.front().descriptor.cols);
    }

    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out)
    {
        return false;
    }

    out.write(kMagic, 4);
    const uint32_t header[4] = {
        kFormatVersion, roleIndex(role), n, d};
    out.write(reinterpret_cast<const char*>(header), sizeof(header));

    const uint32_t elem = kElemFloat32;
    out.write(reinterpret_cast<const char*>(&elem), sizeof(elem));

    std::vector<int32_t> indices;
    indices.reserve(n);
    for (const data::FeatureDescriptor& f : model.features)
    {
        indices.push_back(static_cast<int32_t>(f.point3dIndex));
    }
    if (n > 0)
    {
        out.write(reinterpret_cast<const char*>(indices.data()),
                  static_cast<std::streamsize>(n * sizeof(int32_t)));
    }

    // 描述子数据：**逐条**取该条自己的描述子行。
    //
    // ⚠ ENG-09 §5.19 冻结的 `FeatureDescriptor::descriptor` 是**一条特征
    //    对应的一行**（1×D），不是整个特征集的 N×D 矩阵。若按后者理解，
    //    即把 `features.front().descriptor` 当作全集，则当每条描述子各自
    //    成行（正是本工程各处的构造方式）时，`descriptors.rows == 1`，
    //    于是除第 0 条以外的所有条目都被补零写盘 —— 读回来 N 条都在、
    //    维度也合法，只有内容是 0，**任何一处都不会报错**，最终表现为
    //    "匹配数远低于预期"，与 SYS-12 §18 里"特征库生成有问题"的错误
    //    结论完全同形。故此处不再容忍维度不一致：宁可写失败（返回 false，
    //    调用方据此报错），也不写出一份"看起来完好"的坏库。
    std::vector<float> flat;
    if (n > 0 && d > 0)
    {
        flat.assign(static_cast<size_t>(n) * d, 0.0f);
        for (uint32_t i = 0; i < n; ++i)
        {
            const cv::Mat& row = model.features[i].descriptor;
            if (row.empty() || row.type() != CV_32F
                || row.rows != 1
                || static_cast<uint32_t>(row.cols) != d)
            {
                return false;
            }
            std::memcpy(&flat[static_cast<size_t>(i) * d],
                        row.ptr<float>(0),
                        static_cast<size_t>(d) * sizeof(float));
        }
        out.write(reinterpret_cast<const char*>(flat.data()),
                  static_cast<std::streamsize>(flat.size() * sizeof(float)));
    }
    else if (n > 0)
    {
        // n > 0 而 d == 0：第一条描述子为空，整个库没有可用描述子 ——
        // 头部已按 d=0 写好，但这是调用方不该产出的库（N 条特征却没有
        // 描述子），返回 false 让问题当场暴露。
        return false;
    }

    return out.good();
}

bool TargetModelManager::loadFeatureLibrary(const std::string& path,
                                            data::CameraRole role,
                                            data::TargetModel& inOut)
{
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        return false;
    }

    char magic[4] = {0, 0, 0, 0};
    in.read(magic, 4);
    if (!in || std::memcmp(magic, kMagic, 4) != 0)
    {
        return false;
    }

    uint32_t header[4] = {0, 0, 0, 0};
    in.read(reinterpret_cast<char*>(header), sizeof(header));
    if (!in)
    {
        return false;
    }
    const uint32_t version = header[0];
    const uint32_t storedRole = header[1];
    const uint32_t n = header[2];
    const uint32_t d = header[3];

    uint32_t elem = 0;
    in.read(reinterpret_cast<char*>(&elem), sizeof(elem));

    if (!in || version != kFormatVersion || elem != kElemFloat32)
    {
        return false;
    }

    // 文件名与内容必须自证同源（见文件头 §格式 的说明）。
    data::CameraRole storedRoleEnum = data::CameraRole::CAM25;
    if (!roleFromIndex(storedRole, storedRoleEnum) || storedRoleEnum != role)
    {
        return false;
    }

    std::vector<int32_t> indices(n, 0);
    if (n > 0)
    {
        in.read(reinterpret_cast<char*>(indices.data()),
                static_cast<std::streamsize>(n * sizeof(int32_t)));
        if (!in)
        {
            return false;
        }
    }

    cv::Mat descriptors;
    if (n > 0 && d > 0)
    {
        std::vector<float> flat(static_cast<size_t>(n) * d, 0.0f);
        in.read(reinterpret_cast<char*>(flat.data()),
                static_cast<std::streamsize>(flat.size() * sizeof(float)));
        if (!in)
        {
            return false;
        }
        descriptors = cv::Mat(static_cast<int>(n), static_cast<int>(d),
                              CV_32F, flat.data()).clone();
    }

    inOut.features.clear();
    inOut.features.reserve(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        data::FeatureDescriptor f;
        f.featureId = static_cast<int>(i);
        f.point3dIndex = indices[i];
        if (!descriptors.empty())
        {
            f.descriptor = descriptors.row(static_cast<int>(i)).clone();
        }
        inOut.features.push_back(f);
    }

    return true;
}

// ---------------------------------------------------------------------------

bool TargetModelManager::loadModel(const std::string& modelDir)
{
    // 任何一条失败路径都必须让整体处于"未加载"，不得留下半加载状态。
    loaded_ = false;
    modelId_.clear();
    modelVersion_.clear();
    targetRealSizeM_ = 0.0;
    for (data::TargetModel& m : models_)
    {
        m = data::TargetModel{};
    }

    const std::string pointsPath = joinPath(modelDir, "points3d.yaml");

    std::vector<data::ModelPoint3D> points;
    std::string modelId;
    std::string version;
    if (!loadPoints3d(pointsPath, modelId, version, points))
    {
        return false;
    }

    // ---- model.yaml：模型身份与版本的权威来源（SYS-04 §6.3 / SYS-12 §13）----
    // 缺失即整体加载失败（本文件在 SYS-12 §7 的目录清单中是必需项，
    // 且没有它 model_id 就只能从点表推断 —— 见 loadModelMeta 的说明）。
    if (!loadModelMeta(joinPath(modelDir, "model.yaml"), modelId, version))
    {
        return false;
    }

    // ---- 目标真实尺寸 L：points3d 包围盒的最长边（SYS-14 §10 需要）----
    //
    // ⚠ 最长边为零（全部点共面于一个方向）说明点表退化：此时距离估计式
    // Z = f_x·L/l 的 L 无意义，通道选择会整体错位，故直接判加载失败。
    float minX = points.front().position.x;
    float maxX = minX;
    float minY = points.front().position.y;
    float maxY = minY;
    float minZ = points.front().position.z;
    float maxZ = minZ;
    for (const data::ModelPoint3D& p : points)
    {
        minX = std::min(minX, p.position.x);
        maxX = std::max(maxX, p.position.x);
        minY = std::min(minY, p.position.y);
        maxY = std::max(maxY, p.position.y);
        minZ = std::min(minZ, p.position.z);
        maxZ = std::max(maxZ, p.position.z);
    }
    const double extentX = static_cast<double>(maxX - minX);
    const double extentY = static_cast<double>(maxY - minY);
    const double extentZ = static_cast<double>(maxZ - minZ);
    const double longest = std::max(extentX, std::max(extentY, extentZ));
    if (!(longest > 0.0) || !std::isfinite(longest))
    {
        return false;
    }

    // ---- 三焦段特征库（物理分离，SYS-12 §8）----
    for (data::CameraRole role :
         {data::CameraRole::CAM25, data::CameraRole::CAM50,
          data::CameraRole::CAM100})
    {
        const size_t idx = roleIndex(role);
        data::TargetModel& m = models_[idx];
        m.modelId = modelId;
        m.points3d = points;

        if (!loadFeatureLibrary(joinPath(modelDir, featureFileName(role)),
                                role, m))
        {
            // 一个通道的描述子库缺失 → 整体失败（见头文件说明）。
            loaded_ = false;
            for (data::TargetModel& mm : models_)
            {
                mm = data::TargetModel{};
            }
            return false;
        }
    }

    modelId_ = modelId;
    modelVersion_ = version;
    targetRealSizeM_ = longest;
    loaded_ = true;
    return true;
}

// ---------------------------------------------------------------------------

data::TargetModel TargetModelManager::get(data::CameraRole role) const
{
    if (!loaded_)
    {
        return data::TargetModel{};
    }
    return models_[roleIndex(role)];
}

}  // namespace algorithm
}  // namespace aircraft
