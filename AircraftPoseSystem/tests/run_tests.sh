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
#      APS_SDK_MODE=ON bash tests/run_tests.sh    # 必须有 SDK（缺则失败）
#      APS_SDK_MODE=OFF bash tests/run_tests.sh   # 明确无 SDK 构建树
#
#  APS_SDK_MODE 默认 AUTO（探测，找不到则降级）。旧变量 APS_HAVE_IMVSDK
#  仅作兼容映射 —— 它曾经是**输入**，但会被 Dependencies.cmake 的查找结果
#  覆盖，导致“传了 1 却仍是无 SDK 构建”。现在脚本只信 CMakeCache 里的
#  实际检测值，并在模式与结果不一致时失败。
# ============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build_tests}"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 4)}"

SDK_MODE="${APS_SDK_MODE:-}"
if [[ -z "$SDK_MODE" && -n "${APS_HAVE_IMVSDK:-}" ]]; then
    case "$APS_HAVE_IMVSDK" in
        1|ON|on|TRUE|true)   SDK_MODE=ON ;;
        0|OFF|off|FALSE|false) SDK_MODE=OFF ;;
        *)
            echo "[run_tests] 错误：APS_HAVE_IMVSDK=${APS_HAVE_IMVSDK} 无法识别。" >&2
            exit 2
            ;;
    esac
    echo "[run_tests] 警告：APS_HAVE_IMVSDK 已废弃，按 APS_SDK_MODE=$SDK_MODE 处理" >&2
fi
SDK_MODE="${SDK_MODE:-AUTO}"
case "${SDK_MODE^^}" in
    AUTO|ON|OFF) ;;
    *)
        echo "[run_tests] 错误：APS_SDK_MODE=$SDK_MODE 非法，只接受 AUTO / ON / OFF。" >&2
        exit 2
        ;;
esac
SDK_MODE="${SDK_MODE^^}"

echo "[run_tests] 源码目录：$ROOT"
echo "[run_tests] 构建目录：$BUILD_DIR（APS_SDK_MODE=$SDK_MODE）"

cmake -S "$ROOT" -B "$BUILD_DIR" \
    -DBUILD_TESTING=ON \
    -DAPS_SDK_MODE="$SDK_MODE"

# ---- SDK 分支核对 ----------------------------------------------------------
# 不能只看“传进去的模式”：Dependencies.cmake 里 APS_HAVE_IMVSDK 由
# find_package 结果派生。这里读**实际缓存值**，避免再次出现“声称测了
# SDK 变体、实际跑的是无 SDK 构建”。
ACTUAL_SDK="$(grep -E '^APS_HAVE_IMVSDK:[A-Z]+=' "$BUILD_DIR/CMakeCache.txt" \
    | tail -n 1 | cut -d= -f2)"
if [[ -z "$ACTUAL_SDK" ]]; then
    echo "[run_tests] 错误：CMakeCache.txt 中没有 APS_HAVE_IMVSDK（Dependencies.cmake 版本不匹配？）" >&2
    exit 1
fi
echo "[run_tests] 实际检测：APS_HAVE_IMVSDK=$ACTUAL_SDK（APS_SDK_MODE=$SDK_MODE）"
if [[ "$SDK_MODE" == "ON" && "$ACTUAL_SDK" != "TRUE" ]]; then
    echo "[run_tests] 错误：要求 SDK，但实际未检测到 ImvSdk。" >&2
    exit 1
fi
if [[ "$SDK_MODE" == "OFF" && "$ACTUAL_SDK" == "TRUE" ]]; then
    echo "[run_tests] 错误：APS_SDK_MODE=OFF，但缓存中 APS_HAVE_IMVSDK=TRUE。" >&2
    exit 1
fi

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
