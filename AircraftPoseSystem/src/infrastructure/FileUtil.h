#pragma once

// ============================================================================
//  src/infrastructure/FileUtil.h
//
//  依据：ENG-01 §11（infrastructure 是**唯一**允许做文件 I/O 的层）
//        ENG-10 §4.5（临时文件 + rename 的原子替换）
//
//  作用：infrastructure 内部共享的极小文件/路径工具。**仅头文件、无 .cpp** ——
//  三个函数都是十几行的纯逻辑，单独建一个编译单元只会让链接表多一项，
//  而不会有任何封装收益。
//
//  ⚠ 本文件的存在理由是一次**实测到的真实缺陷**（2026-09-23）：
//
//    `makeDirectories()` 原先在 Logger.cpp / FileMatchStatsStore.cpp /
//    Recorder.cpp 里各有一份，三份的循环体逐字相同，且三份**都漏掉了
//    分隔符**：
//
//        if (c != '/' && i + 1 != path.size()) { current.push_back(c); continue; }
//        if (c != '/') { current.push_back(c); }
//        mkdir(current)                      // ← c == '/' 时既不累加也不补 '/'
//
//    于是 current 在遇到 '/' 后**不再包含分隔符**，下一段被直接拼上去：
//        "output/measurement_x"
//          → mkdir("output")  ✔
//          → mkdir("outputmeasurement_x")  ✘（少了一层，且目录名是错的）
//
//    这个缺陷此前一直潜伏，因为三处的实际入参都只有**一层**
//    （"logs"、"runtime"），循环里从未遇到 '/'。Recorder 写
//    `output/measurement_xxx/`（两层，SYS-04 §6.4）时它立刻暴露：
//    save() 返回 true（mkdir 都成功了），但结果包落在了
//    `outputmeasurement_xxx/` —— 一个**看起来成功、位置错**的结果。
//    这正是"重复代码不会同时被修正"的教科书案例，故此处合并为唯一实现。
//
//    ⚠ 该缺陷的教训值得单独记一笔：它不会让程序崩溃、不会报错、
//    测试里若无"检查目录真的在预期位置"的断言也发现不了。凡是用
//    `mkdir` 返回值判成败的地方，都必须再 `stat` 一次确认**类型**，
//    且调用方必须按**完整路径**断言，而不是只看返回值。
// ============================================================================

#include <cerrno>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

namespace aircraft
{
namespace infrastructure
{
namespace fileutil
{

/// 递归创建目录（等价 `mkdir -p`）。目录已存在视为成功。
///
/// 为什么不用 std::filesystem（C++17 本可直接用 create_directories）：
/// 它在 GCC 9 之前需要额外链接 `stdc++fs`，而本工程的链接表里没有它 ——
/// 在 GCC 12 上虽可直接通过，但换编译器后会在链接期出现一个与本功能
/// 无关的错误。十几行循环换来链接表的确定性，值得。
///
/// 返回值只在**末尾**统一 stat 一次判定：中间层的 mkdir 返回 EEXIST
/// 是常态（上层先建、下层再建），据此判成败会把正常情况当失败。
/// 最终 stat 为目录才算成功 —— 这一条同时挡住了"同名普通文件挡路"
/// 这种 mkdir 返回 EEXIST 却不是目录的情形。
inline bool makeDirectories(const std::string& path)
{
    if (path.empty())
    {
        return false;
    }

    const bool  absolute = path[0] == '/';
    std::string current  = absolute ? "/" : "";
    current.reserve(path.size());

    // 从分隔符后开始累积；每遇到一个 '/'，就把**已经累积出的完整前缀**
    // 建出来，然后把 '/' 也留下 —— 少了这一步，下一段就会与前缀粘连。
    for (std::size_t i = absolute ? 1u : 0u; i < path.size(); ++i)
    {
        const char c = path[i];
        if (c == '/')
        {
            // 绝对路径下 current == "/" 时 size 为 1，不足以构成一层
            if (current.size() > (absolute ? 1u : 0u))
            {
                if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
                {
                    return false;
                }
            }
            current.push_back('/');
            continue;
        }
        current.push_back(c);
    }

    // 最后一段（路径不以 '/' 结尾时的落点；以 '/' 结尾时 current 已含
    // 完整路径，再 mkdir 一次是幂等的）
    if (current.size() > (absolute ? 1u : 0u))
    {
        if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
        {
            return false;
        }
    }

    struct stat st {};
    return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

/// 父目录："runtime/x.yaml" → "runtime"；无 '/' 返回空串；
/// 路径以 '/' 结尾且只有一个 '/' 时返回 "/"。
inline std::string parentDirectory(const std::string& path)
{
    const std::size_t pos = path.find_last_of('/');
    if (pos == std::string::npos)
    {
        return std::string();
    }
    if (pos == 0)
    {
        return "/";
    }
    // 被查找的是"最后一段"的父目录，故末尾的 '/' 本身不算
    if (pos + 1 == path.size())
    {
        return parentDirectory(path.substr(0, pos));
    }
    return path.substr(0, pos);
}

/// 拼接目录与文件名："output" + "result.json" → "output/result.json"。
/// 不使用 '+' 裸拼：漏掉分隔符会得到一个**合法但错误**的路径
/// （"outputresult.json"），与上面 makeDirectories 的缺陷是同一类。
inline std::string joinPath(const std::string& dir, const std::string& name)
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

}  // namespace fileutil
}  // namespace infrastructure
}  // namespace aircraft
