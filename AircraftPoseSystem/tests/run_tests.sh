#!/usr/bin/env bash
# ============================================================================
#  tests/run_tests.sh
#
#  仓库内的**原生测试入口**（C-23 / A5）：配置 → 构建 → CTest。
#
#  与 `/tmp/gtshim/build_tests.sh` 的关系：
#    · 本脚本是首选路径，要求真实 GoogleTest；
#    · 缺少 GoogleTest 时**明确失败并给出安装提示**，不得静默跳过 ——
#      静默跳过会产出一个"0 failed"的空结果，比失败更危险；
#    · `/tmp/gtshim` 仅作为离线回退（垫片不是 GTest，见其文件头）。
#
#  ⚠ 为什么不在脚本里按文件名通配汇总二进制：`/tmp/gtshim` 历史上堆着
#    故意带红的变异件，通配会把它们算进"通过"。CTest 只执行**本次配置
#    注册**的目标，不存在这个风险；本脚本用它作为唯一执行器。
#
#  用法：
#      bash tests/run_tests.sh              # build_tests/ 下配置并全量运行
#      BUILD_DIR=/tmp/aps-build bash tests/run_tests.sh
#      APS_HAVE_IMVSDK=1 bash tests/run_tests.sh   # 需要 SDK 变体时
# ============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build_tests}"
SDK="${APS_HAVE_IMVSDK:-0}"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 4)}"

echo "[run_tests] 源码目录：$ROOT"
echo "[run_tests] 构建目录：$BUILD_DIR（APS_HAVE_IMVSDK=$SDK）"

cmake -S "$ROOT" -B "$BUILD_DIR" \
    -DBUILD_TESTING=ON \
    -DAPS_HAVE_IMVSDK="$SDK"

# ---- GoogleTest 检测 -------------------------------------------------------
# tests/CMakeLists.txt 在缺 GTest 时直接 return()，**不会** add_subdirectory
# 任何测试子目录。故判据是"tests/ 下存在第二层 CTestTestfile.cmake"，
# 而不是"存在任何一个 CTestTestfile"——顶层安装自检
# （install_check / install_launch_check）与 GTest 无关，永远会生成
# build/tests/CTestTestfile.cmake，不能拿来当测试已启用的证据。
if ! find "$BUILD_DIR/tests" -mindepth 2 -name CTestTestfile.cmake \
        -print -quit 2>/dev/null | grep -q .; then
    cat >&2 <<'EOF'

[run_tests] 错误：未找到 GoogleTest，测试目标没有被生成。

  这与"测试全部通过"是两件事：本脚本拒绝在缺少测试的情况下返回 0。
  安装（Ubuntu / UOS 系）：
      sudo apt install libgtest-dev cmake
  或从源码构建后安装到 /usr/local。
  安装完成后重新执行本脚本（无需清理构建目录）。

  离线回退：bash /tmp/gtshim/build_tests.sh（垫片，不是真实 GTest）。
EOF
    exit 1
fi

cmake --build "$BUILD_DIR" -j"$JOBS"

# 只跑 aps_add_test 注册的 `test_*`。install_check / install_launch_check
# 是安装包自检（依赖本机部署环境），不属于本脚本的"测试通过"判据；
# 混进来会让"没装相机"看起来像"测试失败"。
echo "[run_tests] 执行 CTest（test_* 目标）"
ctest --test-dir "$BUILD_DIR" -R "^test_" --output-on-failure --no-tests=error

echo "[run_tests] 全部已注册测试通过"
