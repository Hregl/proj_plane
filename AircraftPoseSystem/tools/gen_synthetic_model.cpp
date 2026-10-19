// ============================================================================
//  tools/gen_synthetic_model.cpp
//
//  用途：把 `data/SyntheticTargetGeometry.h` 中**唯一一份**合成几何写成
//  SYS-12 §7 的机型库目录，供 Qt 仿真交付启动时使用：
//
//      <output_dir>/model.yaml
//      <output_dir>/points3d.yaml
//      <output_dir>/feature25.bin | feature50.bin | feature100.bin
//
//  用法：
//      gen_synthetic_model <output_dir>
//
//  ⚠ 为什么用工具生成而不是把模型文件手工提交：
//    几何只允许有一份定义。手工提交的 model.yaml / points3d.yaml 会在
//    `SyntheticTargetGeometry.h` 改动后静默过期 —— 图像按新几何渲染、
//    PnP 却按旧点表求解，重投影仍可能很小，但姿态与真值无关。
//    每次启动都由本工具从同一头文件重新生成，模型与画面不会分叉。
//
//  ⚠ 本工具不读取真值、不写结果包，也不进入生产启动路径；它只是
//    SYS-12 §12 所述离线模型生成器在本批的最小实现。
// ============================================================================

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "algorithm/model/TargetModelManager.h"
#include "data/CameraRole.h"
#include "data/SyntheticTargetGeometry.h"
#include "data/TargetModel.h"

namespace
{

bool writeText(const std::filesystem::path& path, const std::string& text)
{
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out)
    {
        std::cerr << "写文件失败：" << path << "\n";
        return false;
    }
    out << text;
    if (!out.good())
    {
        std::cerr << "写文件失败：" << path << "\n";
        return false;
    }
    return true;
}

std::string modelYaml()
{
    std::ostringstream out;
    out << "%YAML:1.0\n"
        << "# 由 tools/gen_synthetic_model 从 data/SyntheticTargetGeometry.h 生成。\n"
        << "# 不要手工编辑：启动脚本每次运行都会覆盖本目录。\n"
        << "model_id: \"" << aircraft::data::synthetic::modelId() << "\"\n"
        << "version: 1\n"
        << "model_source: \"data/SyntheticTargetGeometry.h\"\n"
        << "feature_algorithm_version: \"none\"\n";
    return out.str();
}

std::string pointsYaml(const std::vector<aircraft::data::ModelPoint3D>& points)
{
    std::ostringstream out;
    out << "%YAML:1.0\n"
        << "model_id: \"" << aircraft::data::synthetic::modelId() << "\"\n"
        << "version: 1\n"
        << "points:\n";
    out.setf(std::ios::fixed);
    out.precision(6);
    for (const aircraft::data::ModelPoint3D& p : points)
    {
        out << "  - { id: " << p.id << ", x: " << p.position.x
            << ", y: " << p.position.y << ", z: " << p.position.z
            << ", type: " << p.featureType << " }\n";
    }
    return out.str();
}

}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 2 || argv[1] == nullptr || argv[1][0] == '\0')
    {
        std::cerr << "用法: gen_synthetic_model <output_dir>\n";
        return 2;
    }

    const std::filesystem::path outDir(argv[1]);
    std::error_code ec;
    std::filesystem::create_directories(outDir, ec);
    if (ec)
    {
        std::cerr << "创建目录失败：" << outDir << "：" << ec.message() << "\n";
        return 1;
    }

    const std::vector<aircraft::data::ModelPoint3D> points =
        aircraft::data::synthetic::modelPoints();

    if (!writeText(outDir / "model.yaml", modelYaml())
        || !writeText(outDir / "points3d.yaml", pointsYaml(points)))
    {
        return 1;
    }

    // 三个焦段的特征库：本场景是 B 类 CAD 结构点通路，A 类纹理库为空
    // （N=0、D=0 是格式允许的“尚无纹理特征”状态）。空库也必须写出，
    // 因为 TargetModelManager 把三个 .bin 都视为必需文件。
    aircraft::data::TargetModel model;
    model.modelId = aircraft::data::synthetic::modelId();
    model.points3d = points;

    const aircraft::data::CameraRole roles[3] = {
        aircraft::data::CameraRole::CAM25,
        aircraft::data::CameraRole::CAM50,
        aircraft::data::CameraRole::CAM100};
    const char* const names[3] = {
        "feature25.bin", "feature50.bin", "feature100.bin"};

    for (int i = 0; i < 3; ++i)
    {
        const std::filesystem::path path = outDir / names[i];
        if (!aircraft::algorithm::TargetModelManager::saveFeatureLibrary(
                path.string(), roles[i], model))
        {
            std::cerr << "写特征库失败：" << path << "\n";
            return 1;
        }
    }

    std::cout << "合成机型库已生成：" << outDir << "（"
              << points.size() << " 个 CAD 结构点，三个空纹理库）\n";
    return 0;
}
