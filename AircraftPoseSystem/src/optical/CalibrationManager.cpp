// ============================================================================
//  src/optical/CalibrationManager.cpp
//
//  依据：ENG-09 §2.3（毫米→米转换必须在本文件完成）、§5.2 / §5.3
//        ENG-01 §3.4（7 个标定文件）、ENG-10 §5.3（三种装载失败模式）
//
//  ⚠ 本文件是全工程唯一的**毫米→米**边界。详见头文件的分析。
//
//  ⚠ 标定文件的 YAML 字段结构**未被任何冻结文档定义**
//  （ENG-01 §3.4 只给了文件名，未给内容 schema）。
//  因此本文件定义的 schema 是本工程的**约定**，不是冻结接口 ——
//  它必须与标定工具（SYS-11）的实际输出保持一致，二者不一致时
//  以实际输出为准并回改本文件。schema 见下方 kField* 常量处的说明。
//
//  特别注意平移字段命名为 `translation_mm` 而非 `translation`：
//  让**单位出现在文件名/字段名里**，使"这个数是什么单位"不需要靠
//  记忆或注释来回答。这是 ENG-09 §2.3 那条转换规则在文件格式上的
//  直接体现 —— 若字段只叫 translation，读文件的人无从判断该不该除以
//  1000，而先按毫米写、后被按米读取的标定文件会静默产生 1000 倍误差。
// ============================================================================

#include "optical/CalibrationManager.h"

#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>

namespace aircraft
{
namespace optical
{

namespace
{
// ---- 标定文件字段名（本工程约定，见文件头）----
constexpr const char* kFieldImageWidth   = "image_width";
constexpr const char* kFieldImageHeight  = "image_height";
constexpr const char* kFieldCameraMatrix = "camera_matrix";
constexpr const char* kFieldDistortion   = "distortion";
constexpr const char* kFieldRotation     = "rotation";
constexpr const char* kFieldTranslationMm = "translation_mm";  // ⚠ 毫米
constexpr const char* kFieldCalibrationId = "calibration_id";

/// 标定文件的 7 个文件名（ENG-01 §3.4，逐字冻结）。
/// 命名遵循 ENG-09 §2.1 的 `AToB` 方向约定：`cam25_to_rig` 表示
/// Camera → OpticalRig，即 p_rig = cam25_to_rig · p_camera。
constexpr const char* kFileIntrinsic[3] = {"cam25.yaml", "cam50.yaml", "cam100.yaml"};
constexpr const char* kFileExtrinsic[3] = {"cam25_to_rig.yaml",
                                           "cam50_to_rig.yaml",
                                           "cam100_to_rig.yaml"};
constexpr const char* kFileRigToShip    = "rig_to_ship.yaml";

/// 毫米→米。全工程唯一的换算因子，只在本文件使用。
constexpr double kMmToM = 1.0 / 1000.0;

/// 合成内参的标称水平视场角，单位 deg。
///
/// ⚠ 这是一个**占位假设**，不是任何真实镜头的参数。
/// 它只用于让合成内参在数值上合理（焦距为正、主点在画幅内），
/// 从而预览与算法链的数据通路可以被完整跑通。
/// 由此算出的任何姿态结果都**没有物理意义** —— 见
/// CalibrationManager::loadDefaults() 的说明。
constexpr double kSyntheticHorizontalFovDeg = 45.0;

/// 数值校验的容差。标定文件经 YAML 文本往返后仍有 ~1e-6 的相对误差，
/// 而把欧拉角误写成矩阵（R10 要拦的情形）误差量级为 1e-1，两者相距很远。
constexpr double kRotationTolerance = 1e-3;
constexpr double kHomogeneousTolerance = 1e-6;

bool allFinite(const cv::Mat& m64)
{
    for (int r = 0; r < m64.rows; ++r)
    {
        const double* row = m64.ptr<double>(r);
        for (int c = 0; c < m64.cols; ++c)
        {
            if (!std::isfinite(row[c]))
            {
                return false;
            }
        }
    }
    return true;
}

/// 把 cv::Mat 读成 3x3 旋转矩阵并校验有限性/正交性/det≈+1。
///
/// R10：非法旋转**拒绝装载**，不再回退单位阵。回退单位阵会把一个
/// 格式错误的标定变成"看起来有效"的单位外参，而所有下游数值仍然合理。
bool readRotation(const cv::FileStorage& fs,
                  const std::string&  context,
                  cv::Matx33d&        out,
                  std::string&        error)
{
    cv::Mat m;
    fs[kFieldRotation] >> m;

    if (m.empty() || m.rows != 3 || m.cols != 3 || m.channels() != 1)
    {
        error = context + "：rotation 缺失或不是单通道 3x3";
        return false;
    }

    cv::Mat m64;
    m.convertTo(m64, CV_64F);
    if (!allFinite(m64))
    {
        error = context + "：rotation 含非有限值";
        return false;
    }

    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            out(r, c) = m64.at<double>(r, c);
        }
    }

    const cv::Matx33d shouldBeIdentity = out * out.t();
    double maxOrthoError = 0.0;
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            const double expected = (r == c) ? 1.0 : 0.0;
            maxOrthoError =
                std::max(maxOrthoError,
                         std::fabs(shouldBeIdentity(r, c) - expected));
        }
    }
    if (maxOrthoError > kRotationTolerance)
    {
        error = context + "：rotation 不是正交矩阵（max |R·Rᵀ−I| = " +
                std::to_string(maxOrthoError) + "）";
        return false;
    }

    const double det = cv::determinant(cv::Mat(out));
    if (std::fabs(det - 1.0) > kRotationTolerance)
    {
        error = context + "：rotation 的 det 不是 +1（det = " +
                std::to_string(det) + "）";
        return false;
    }
    return true;
}

/// 读平移量并**在此处**完成毫米→米换算。
///
/// 这是 ENG-09 §2.3 要求的唯一转换点。整个工程只有这一个地方
/// 出现 `* kMmToM` 作用在标定平移上。
///
/// R10：缺失/形状错误/非有限值一律拒绝装载，不再回退零平移。
bool readTranslationMetres(const cv::FileStorage& fs,
                           const std::string&  context,
                           cv::Vec3d&          out,
                           std::string&        error)
{
    cv::Mat t;
    fs[kFieldTranslationMm] >> t;

    if (t.empty() || t.total() != 3 || t.channels() != 1)
    {
        error = context + "：translation_mm 缺失或不是 3 个元素";
        return false;
    }

    cv::Mat t64;
    t.convertTo(t64, CV_64F);
    if (!allFinite(t64))
    {
        error = context + "：translation_mm 含非有限值";
        return false;
    }

    const double* p = t64.ptr<double>();

    // ⚠⚠ 毫米 → 米。此处是全工程唯一的换算点（ENG-09 §2.3）。
    out = cv::Vec3d(p[0] * kMmToM, p[1] * kMmToM, p[2] * kMmToM);
    return true;
}

/// 内参 + 畸变的装载校验（R10）。
///
/// 判据是**数值/结构**的，不是"正定"：
///   · image_width/height > 0（畸变与主点的坐标一致性需要分辨率）；
///   · camera_matrix 单通道 3x3、全有限、fx>0、fy>0；
///   · 齐次末行 [0,0,1]（容差 1e-6）；
///   · 主点在画幅内（cx∈[0,W)、cy∈[0,H)）——这是"内参与图像坐标一致"
///     的最小可执行判据；
///   · distortion 缺失 ⇒ 警告并按无畸变（针孔）处理；存在则必须是
///     1×N / N×1（N∈{4,5,8}）且全有限。
bool loadIntrinsic(const cv::FileStorage& fs,
                   const std::string&  context,
                   data::CameraCalibration& calib,
                   std::vector<std::string>& warnings,
                   std::string&        error)
{
    fs[kFieldImageWidth]  >> calib.imageWidth;
    fs[kFieldImageHeight] >> calib.imageHeight;
    if (calib.imageWidth <= 0 || calib.imageHeight <= 0)
    {
        error = context + "：image_width/height 缺失或非正，无法校验内参一致性";
        return false;
    }

    cv::Mat k;
    fs[kFieldCameraMatrix] >> k;
    if (k.empty() || k.rows != 3 || k.cols != 3 || k.channels() != 1)
    {
        error = context + "：camera_matrix 缺失或不是单通道 3x3";
        return false;
    }

    cv::Mat k64;
    k.convertTo(k64, CV_64F);
    if (!allFinite(k64))
    {
        error = context + "：camera_matrix 含非有限值";
        return false;
    }

    const double fx = k64.at<double>(0, 0);
    const double fy = k64.at<double>(1, 1);
    if (!(fx > 0.0) || !(fy > 0.0))
    {
        error = context + "：camera_matrix 的 fx/fy 必须为正（fx=" +
                std::to_string(fx) + ", fy=" + std::to_string(fy) + "）";
        return false;
    }

    if (std::fabs(k64.at<double>(2, 0)) > kHomogeneousTolerance ||
        std::fabs(k64.at<double>(2, 1)) > kHomogeneousTolerance ||
        std::fabs(k64.at<double>(2, 2) - 1.0) > kHomogeneousTolerance)
    {
        error = context + "：camera_matrix 的齐次末行不是 [0,0,1]";
        return false;
    }

    const double cx = k64.at<double>(0, 2);
    const double cy = k64.at<double>(1, 2);
    if (cx < 0.0 || cx >= static_cast<double>(calib.imageWidth) ||
        cy < 0.0 || cy >= static_cast<double>(calib.imageHeight))
    {
        error = context + "：主点不在画幅内（cx=" + std::to_string(cx) +
                ", cy=" + std::to_string(cy) + "；画幅 " +
                std::to_string(calib.imageWidth) + "x" +
                std::to_string(calib.imageHeight) + "）";
        return false;
    }
    calib.cameraMatrix = k64;

    cv::Mat d;
    fs[kFieldDistortion] >> d;
    if (d.empty())
    {
        // 无畸变系数 = 假定理想针孔。这不是致命的，但必须记录：
        // 无视畸变会让边缘视场的反投影误差随视场角增大。
        warnings.push_back(context + "：distortion 缺失，按无畸变处理");
        calib.distortion = cv::Mat::zeros(1, 5, CV_64F);
        return true;
    }

    if (d.channels() != 1 || (d.rows != 1 && d.cols != 1) ||
        (d.total() != 4 && d.total() != 5 && d.total() != 8))
    {
        error = context + "：distortion 形状非法（须为 1×N/N×1，N∈{4,5,8}）";
        return false;
    }

    cv::Mat d64;
    d.convertTo(d64, CV_64F);
    if (!allFinite(d64))
    {
        error = context + "：distortion 含非有限值";
        return false;
    }
    calib.distortion = d64.reshape(1, 1);  // 统一成 1×N，便于下游使用
    return true;
}

}  // namespace

// ---------------------------------------------------------------------------

bool CalibrationManager::load(const std::string& calibrationDir)
{
    calibration_   = data::OpticalRigCalibration{};
    calibrationId_.clear();
    warnings_.clear();
    lastErrorText_.clear();
    loaded_ = false;

    if (calibrationDir.empty())
    {
        lastErrorText_ = "标定目录为空（OpticalRigConfig::calibrationDir 未配置）";
        return false;
    }

    // 目录分隔符：不以 '/' 结尾时补一个，避免拼出 "dir" + "cam25.yaml"
    // 这种缺分隔符的路径（表现为"文件不存在"，而文件其实存在）。
    std::string dir = calibrationDir;
    if (dir.back() != '/')
    {
        dir += '/';
    }

    data::CameraCalibration* intrinsics[3] = {
        &calibration_.cam25, &calibration_.cam50, &calibration_.cam100};
    data::Transform* extrinsics[3] = {
        &calibration_.cam25.cameraToRig,
        &calibration_.cam50.cameraToRig,
        &calibration_.cam100.cameraToRig};
    const char* roleName[3] = {"CAM25", "CAM50", "CAM100"};

    // R10：任何非法标定都拒绝装载，且不保留半装载状态。
    auto fail = [this](const std::string& message) {
        lastErrorText_   = message;
        calibration_     = data::OpticalRigCalibration{};
        calibrationId_.clear();
        loaded_          = false;
        return false;
    };

    // ---- 1 内参 ----
    for (int i = 0; i < 3; ++i)
    {
        const std::string path = dir + kFileIntrinsic[i];

        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            // ENG-10 §5.3 第一类：文件缺失 → 启动失败。
            return fail("标定文件缺失或无法打开：" + path);
        }

        const std::string ctx = std::string(roleName[i]) + " 内参 (" +
                                kFileIntrinsic[i] + ")";

        std::string error;
        if (!loadIntrinsic(fs, ctx, *intrinsics[i], warnings_, error))
        {
            return fail(error);
        }

        // 内参矩阵的数值单位是**像素**，不涉及毫米→米转换。
        // 这一点容易与平移量混淆：镜头焦距以米表达（CameraChannel
        // ::focalLength），而内参矩阵中的 fx/fy 是以像素表达的焦距，
        // 二者通过像元尺寸相联系。ENG-09 §2.3 的换算规则**不适用于**
        // 内参矩阵，误对它除以 1000 会让所有投影退化到主点附近。
    }

    // ---- 2 外参 Camera → Rig ----
    for (int i = 0; i < 3; ++i)
    {
        const std::string path = dir + kFileExtrinsic[i];

        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            return fail("标定文件缺失或无法打开：" + path);
        }

        const std::string ctx = std::string(roleName[i]) + " 外参 (" +
                                kFileExtrinsic[i] + ")";

        std::string error;
        cv::Matx33d rotation;
        cv::Vec3d translation;
        if (!readRotation(fs, ctx, rotation, error) ||
            !readTranslationMetres(fs, ctx, translation, error))
        {
            return fail(error);
        }
        extrinsics[i]->rotation    = rotation;
        extrinsics[i]->translation = translation;
    }

    // ---- 3 外参 Rig → Ship ----
    {
        const std::string path = dir + kFileRigToShip;

        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            return fail("标定文件缺失或无法打开：" + path);
        }

        const std::string ctx = std::string("Rig→Ship 外参 (") + kFileRigToShip + ")";

        std::string error;
        cv::Matx33d rotation;
        cv::Vec3d translation;
        if (!readRotation(fs, ctx, rotation, error) ||
            !readTranslationMetres(fs, ctx, translation, error))
        {
            return fail(error);
        }
        calibration_.rigToShip.rotation    = rotation;
        calibration_.rigToShip.translation = translation;

        // 标定版本标识：ENG-09 §6.8 要求写入结果包以支持离线复现。
        // 它在**标定文件**里而非仅配置里，使"文件内容与标识"绑定 ——
        // 若只信配置，则标定目录被整包替换而配置未更新时，
        // 结果包会记录一个与实际数据不符的版本号。
        std::string id;
        fs[kFieldCalibrationId] >> id;
        if (!id.empty())
        {
            calibrationId_ = id;
        }
    }

    loaded_ = true;
    return true;
}

bool CalibrationManager::loadDefaults(int imageWidth, int imageHeight)
{
    calibration_ = data::OpticalRigCalibration{};
    warnings_.clear();
    lastErrorText_.clear();

    const int w = imageWidth  > 0 ? imageWidth  : 1280;
    const int h = imageHeight > 0 ? imageHeight : 1024;

    calibration_.cam25  = makeSyntheticCalibration(w, h);
    calibration_.cam50  = makeSyntheticCalibration(w, h);
    calibration_.cam100 = makeSyntheticCalibration(w, h);

    // 外参：单位变换 —— 即假定三台相机光轴平行、且与舰体坐标轴对齐。
    // 这是**虚构的**，但它使坐标链的所有环节都能被完整执行，
    // 从而预览、日志、结果落盘这些与精度无关的通路可以被验证。
    calibration_.cam25.cameraToRig  = data::Transform{};
    calibration_.cam50.cameraToRig  = data::Transform{};
    calibration_.cam100.cameraToRig = data::Transform{};
    calibration_.rigToShip          = data::Transform{};

    calibrationId_ = "SYNTHETIC-NO-CALIBRATION";

    warnings_.push_back(
        "正在使用**合成标定**：内参为标称视场角反推的占位值，"
        "外参为单位变换。由其算出的任何姿态结果都没有物理意义，"
        "仅可用于数据通路调试（M1/M2）。");

    loaded_ = true;
    return true;
}

data::CameraCalibration CalibrationManager::getCalibration(data::CameraRole role) const
{
    switch (role)
    {
    case data::CameraRole::CAM25:
        return calibration_.cam25;
    case data::CameraRole::CAM50:
        return calibration_.cam50;
    case data::CameraRole::CAM100:
        return calibration_.cam100;
    }
    // CameraRole 只有三个取值（ENG-09 §4.1），此分支不可达。
    // 5.md §八 里此处的 default 返回默认构造的 CameraCalibration ——
    // 本实现保留该行为作为兜底，但**没有**把它当作正常路径。
    return data::CameraCalibration{};
}

data::Transform CalibrationManager::getRigToShip() const
{
    return calibration_.rigToShip;
}

data::OpticalRigCalibration CalibrationManager::calibration() const
{
    return calibration_;
}

std::string CalibrationManager::calibrationId() const
{
    return calibrationId_;
}

bool CalibrationManager::loaded() const
{
    return loaded_;
}

std::vector<std::string> CalibrationManager::lastWarnings() const
{
    return warnings_;
}

std::string CalibrationManager::lastErrorText() const
{
    return lastErrorText_;
}

// ---------------------------------------------------------------------------

data::CameraCalibration CalibrationManager::makeSyntheticCalibration(int imageWidth,
                                                                    int imageHeight)
{
    data::CameraCalibration c;

    c.imageWidth  = imageWidth;
    c.imageHeight = imageHeight;

    // 由标称水平视场角反推焦距（像素）：
    //     fx = (W / 2) / tan(HFOV / 2)
    // 这是一个**占位**公式。真实内参必须来自 SYS-11 的标定流程。
    constexpr double kPi = 3.14159265358979323846;
    const double halfFovRad = (kSyntheticHorizontalFovDeg * kPi / 180.0) / 2.0;
    const double fx = (static_cast<double>(imageWidth) / 2.0) / std::tan(halfFovRad);
    const double fy = fx;  // 假定像元为正方形

    // 主点取画幅中心。
    const double cx = static_cast<double>(imageWidth) / 2.0;
    const double cy = static_cast<double>(imageHeight) / 2.0;

    c.cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0.0, cx,
                                                0.0, fy, cy,
                                                0.0, 0.0, 1.0);

    // 无畸变。
    c.distortion = cv::Mat::zeros(1, 5, CV_64F);

    c.cameraToRig = data::Transform{};

    return c;
}

}  // namespace optical
}  // namespace aircraft
