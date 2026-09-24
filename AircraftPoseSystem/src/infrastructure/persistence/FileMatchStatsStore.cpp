// ============================================================================
//  src/infrastructure/persistence/FileMatchStatsStore.cpp
//
//  读：cv::FileStorage（与 config/*.yaml、标定文件同一套解析器）。
//  写：手写文本 + 临时文件 + rename（ENG-10 §4.5 的原子替换）。
//
//  ⚠ 为何写不用 FileStorage：本文件每次 save 都要整体重写，用
//    FileStorage 写"嵌套序列 + 映射"需要手工拼 "[" / "{" 标记，
//    出错时得到的是一个**结构错位但语法合法**的文件（下次能打开、
//    内容却错行）—— 这种错误要到几个月后回放数据时才暴露。
//    手写文本的格式完全由本文件决定，且 parse 侧仍由 FileStorage
//    校验（写错格式会在下次 load 时立刻暴露，而不是静默接受）。
//
//  ⚠ `%YAML:1.0` 必须在**首行**（实测：OpenCV 要求如此，指令前有注释
//    即抛 "Input file is invalid"）。本文件生成的文本首行即该指令。
// ============================================================================

#include "infrastructure/persistence/FileMatchStatsStore.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include <opencv2/core.hpp>

#include "data/MonotonicClock.h"
#include "infrastructure/FileUtil.h"

namespace aircraft
{
namespace infrastructure
{

// makeDirectories() / parentDir() 原在此处各有一份（前者与 Logger.cpp、
// Recorder.cpp 逐字重复，且三份都漏掉了路径分隔符）。已合并为
// infrastructure/FileUtil.h 中的唯一实现，说明见该文件头。
using fileutil::makeDirectories;
using fileutil::parentDirectory;

// ---------------------------------------------------------------------------

FileMatchStatsStore::FileMatchStatsStore(std::string path,
                                        const data::MeasurementConfig& config)
    : path_(std::move(path))
    , minSamples_(static_cast<double>(std::max(1, config.matchStatsMinSamples)))
    , coldStartPrior_(config.matchStatsColdStartPrior)
{
    if (!std::isfinite(coldStartPrior_) || coldStartPrior_ < 0.0 ||
        coldStartPrior_ > 1.0)
    {
        // 先验越界会让 M 分项失去意义（>1 时"冷启动的相机"反而占优）。
        // ConfigManager 已按 ENG-10 §5.3 拦截，此处只是兜底。
        coldStartPrior_ = 0.5;
    }
}

// ---------------------------------------------------------------------------

int FileMatchStatsStore::roleIndexOf(int roleValue)
{
    switch (roleValue)
    {
    case 0: return 0;  // CameraRole::CAM25 的枚举序号
    case 1: return 1;  // CAM50
    case 2: return 2;  // CAM100
    case 25:  return 0;   // 接口注释的写法（mm）
    case 50:  return 1;
    case 100: return 2;
    default:  return -1;
    }
}

int FileMatchStatsStore::bucketIndex(int role, int distanceBand, int illumBand)
{
    const int r = roleIndexOf(role);
    if (r < 0 || distanceBand < 0 || distanceBand >= kDistanceBands ||
        illumBand < 0 || illumBand >= kIllumBands)
    {
        return -1;
    }
    return r * (kDistanceBands * kIllumBands) + distanceBand * kIllumBands +
           illumBand;
}

double FileMatchStatsStore::successRate(const std::string& targetModelId,
                                       int role,
                                       int distanceBand,
                                       int illumBand) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    const int index = bucketIndex(role, distanceBand, illumBand);
    if (index < 0)
    {
        // 接口契约：索引越界返回冷启动先验，不抛出。
        // 调用链（MeasurementSelector）已按同一份 config 算过索引，
        // 越界说明配置与调用点不一致 —— 返回先验让流程继续，
        // 而不会让 NaN 参与评分（NaN 会让比较恒为 false，
        // 选择结果静默退化为"第一个候选"）。
        return coldStartPrior_;
    }

    const auto it = table_.find(targetModelId);
    if (it == table_.end())
    {
        return coldStartPrior_;   // 该机型的表为空 = 冷启动
    }

    const Bucket& b = it->second[static_cast<std::size_t>(index)];
    if (b.attempts == 0)
    {
        return coldStartPrior_;
    }

    // ENG-10 §4.3 的收缩估计（经验贝叶斯）：
    //     M = (n·p̂ + N_min·prior) / (n + N_min)
    // 不用硬阈值截断（"不足 N_min 次就当作先验"）的原因：那会让第 9 次
    // 与第 10 次观测之间出现结果跳变，而两次的数据其实只差一条。
    const double n    = static_cast<double>(b.attempts);
    const double pHat = static_cast<double>(b.successes) / n;
    return (n * pHat + minSamples_ * coldStartPrior_) / (n + minSamples_);
}

void FileMatchStatsStore::record(const std::string& targetModelId,
                                 int role,
                                 int distanceBand,
                                 int illumBand,
                                 bool success)
{
    const int index = bucketIndex(role, distanceBand, illumBand);
    if (index < 0)
    {
        // 与 successRate 不同，写入时的越界**不能**静默忽略：
        // 忽略会让一次观测凭空消失，而统计量的偏差不会被任何日志发现。
        // 但本类无 Logger（infrastructure 内部不允许反向依赖），
        // 故记入 lastErrorText_ 由调用方在 save 时检查。
        std::lock_guard<std::mutex> lock(mutex_);
        std::ostringstream os;
        os << "record 的分桶索引越界：role=" << role
           << " distanceBand=" << distanceBand << " illumBand=" << illumBand
           << "（观测被丢弃）";
        lastErrorText_ = os.str();
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto it = table_.find(targetModelId);
    if (it == table_.end())
    {
        it = table_.emplace(targetModelId,
                            std::vector<Bucket>(kBucketCount))
                 .first;
    }

    Bucket& b = it->second[static_cast<std::size_t>(index)];
    ++b.attempts;
    if (success)
    {
        ++b.successes;
    }
    dirty_ = true;
}

// ---------------------------------------------------------------------------

std::string FileMatchStatsStore::serializeLocked() const
{
    std::ostringstream os;
    os << "%YAML:1.0\n";          // 必须首行，见文件头说明
    os << "---\n";
    os << "match_stats:\n";
    os << "   version: 1\n";
    os << "   models:\n";

    for (const auto& entry : table_)
    {
        os << "      -\n";
        os << "         model_id: \"" << entry.first << "\"\n";
        os << "         buckets:\n";
        for (int i = 0; i < kBucketCount; ++i)
        {
            const Bucket& b = entry.second[static_cast<std::size_t>(i)];
            if (b.attempts == 0)
            {
                continue;   // 空桶不写：表的大小随实际观测增长
            }
            const int role = i / (kDistanceBands * kIllumBands);
            const int rem  = i % (kDistanceBands * kIllumBands);
            os << "            -\n";
            os << "               role: " << role << "\n";
            os << "               distance_band: " << (rem / kIllumBands) << "\n";
            os << "               illum_band: " << (rem % kIllumBands) << "\n";
            os << "               attempts: " << b.attempts << "\n";
            os << "               successes: " << b.successes << "\n";
        }
    }
    return os.str();
}

bool FileMatchStatsStore::save()
{
    std::lock_guard<std::mutex> lock(mutex_);

    const std::string dir = parentDirectory(path_);
    if (!dir.empty() && !makeDirectories(dir))
    {
        lastErrorText_ = "无法创建目录：" + dir;
        return false;
    }

    // ENG-10 §4.5：临时文件 + rename（原子替换）。
    // 直接覆盖写的话，写一半掉电会留下一个既非旧值也非新值的文件 ——
    // 而它是下次启动的输入，统计表就此损坏且无从判断。
    const std::string tmp = path_ + ".tmp";
    {
        std::ofstream out(tmp, std::ios::out | std::ios::trunc);
        if (!out.is_open())
        {
            lastErrorText_ = "无法写入临时文件：" + tmp;
            return false;
        }
        out << serializeLocked();
        out.flush();
        if (!out.good())
        {
            lastErrorText_ = "写入临时文件失败（磁盘满？）：" + tmp;
            out.close();
            ::unlink(tmp.c_str());
            return false;
        }
    }

    if (::rename(tmp.c_str(), path_.c_str()) != 0)
    {
        lastErrorText_ = "rename 失败：" + tmp + " → " + path_;
        ::unlink(tmp.c_str());
        return false;
    }

    dirty_ = false;
    lastErrorText_.clear();
    return true;
}

bool FileMatchStatsStore::load()
{
    std::lock_guard<std::mutex> lock(mutex_);

    table_.clear();
    loadedFromFile_ = false;
    lastCorruptPath_.clear();

    std::ifstream probe(path_);
    if (!probe.good())
    {
        // 文件不存在 = 冷启动，属**正常**情况（首次运行）：
        // 返回 true，让调用方不必为"还没有历史统计"报错。
        // 与"解析失败"必须区分开 —— 后者的返回值为 false。
        lastErrorText_.clear();
        return true;
    }
    probe.close();

    cv::FileStorage fs;
    bool            parseFailed = false;
    std::string     why;

    try
    {
        fs.open(path_, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            parseFailed = true;
            why         = "无法打开";
        }
    }
    catch (const cv::Exception& e)
    {
        parseFailed = true;
        why         = e.what();
    }

    if (!parseFailed)
    {
        const cv::FileNode models = fs["match_stats"]["models"];
        if (!models.isSeq())
        {
            parseFailed = true;
            why         = "match_stats.models 不是序列";
        }
        else
        {
            for (auto it = models.begin(); it != models.end(); ++it)
            {
                const cv::FileNode node = *it;
                const cv::FileNode idNode = node["model_id"];
                if (!idNode.isString())
                {
                    parseFailed = true;
                    why         = "条目缺少 model_id";
                    break;
                }
                const std::string modelId = idNode.string();
                std::vector<Bucket> buckets(kBucketCount);

                const cv::FileNode list = node["buckets"];
                if (list.isSeq())
                {
                    for (auto bit = list.begin(); bit != list.end(); ++bit)
                    {
                        const cv::FileNode b = *bit;
                        const int index = bucketIndex(
                            static_cast<int>(b["role"].real()),
                            static_cast<int>(b["distance_band"].real()),
                            static_cast<int>(b["illum_band"].real()));
                        if (index < 0)
                        {
                            // 单桶索引损坏：**跳过该桶**而不是放弃整表。
                            // 一个桶的键写错不该让其余 35 个桶的历史
                            // 统计一起丢失。
                            continue;
                        }
                        const double att = b["attempts"].real();
                        const double suc = b["successes"].real();
                        if (!(att >= 0.0) || !(suc >= 0.0) || suc > att)
                        {
                            continue;   // 计数非法：留空桶（= 冷启动）
                        }
                        Bucket& target = buckets[static_cast<std::size_t>(index)];
                        target.attempts  = static_cast<uint64_t>(att);
                        target.successes = static_cast<uint64_t>(suc);
                    }
                }

                table_.emplace(modelId, std::move(buckets));
            }
        }
    }

    if (parseFailed)
    {
        // ENG-10 §4.6：改名保留现场，从空表重启，**不得静默丢弃**。
        // 静默丢弃的后果是统计量无声退化到冷启动先验，而使用者无从知道
        // （表现只是"选择结果变得跟以前不一样"，没有任何错误码）。
        table_.clear();
        lastCorruptPath_ = path_ + ".corrupt." +
                           std::to_string(data::monotonicNowNs());
        if (::rename(path_.c_str(), lastCorruptPath_.c_str()) != 0)
        {
            lastCorruptPath_.clear();   // 改名失败也要继续（不能因此不启动）
        }
        lastErrorText_ = "match_stats 解析失败（" + why +
                         "），已改名保留现场并从空表重启";
        dirty_ = false;
        return false;
    }

    loadedFromFile_ = true;
    dirty_          = false;
    lastErrorText_.clear();
    return true;
}

// ---------------------------------------------------------------------------

bool FileMatchStatsStore::loadedFromFile() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return loadedFromFile_;
}

std::size_t FileMatchStatsStore::modelCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return table_.size();
}

std::size_t FileMatchStatsStore::nonEmptyBucketCount(
    const std::string& targetModelId) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = table_.find(targetModelId);
    if (it == table_.end())
    {
        return 0;
    }
    return static_cast<std::size_t>(std::count_if(
        it->second.begin(), it->second.end(),
        [](const Bucket& b) { return b.attempts > 0; }));
}

uint64_t FileMatchStatsStore::attempts(const std::string& targetModelId,
                                      int role,
                                      int distanceBand,
                                      int illumBand) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const int index = bucketIndex(role, distanceBand, illumBand);
    if (index < 0)
    {
        return 0;
    }
    const auto it = table_.find(targetModelId);
    if (it == table_.end())
    {
        return 0;
    }
    return it->second[static_cast<std::size_t>(index)].attempts;
}

std::string FileMatchStatsStore::lastErrorText() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return lastErrorText_;
}

std::string FileMatchStatsStore::lastCorruptPath() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return lastCorruptPath_;
}

bool FileMatchStatsStore::dirty() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return dirty_;
}

}  // namespace infrastructure
}  // namespace aircraft
