#!/usr/bin/env bash
# ============================================================================
#  scripts/run_simulation.sh  —— Qt 仿真交付的可复现启动入口
#
#  用途：在**不接任何真实设备**的条件下启动 AircraftPoseSystem：
#    1. 用独立的无 SDK 构建树编译（默认 build-simulation，可被
#       APS_SIM_BUILD_DIR 覆盖）；
#    2. 从 data/SyntheticTargetGeometry.h 重新生成仿真机型库
#       （simulation/models/，每次启动覆盖，避免模型与画面几何分叉）；
#    3. 以 simulation/config/ 启动主界面。
#
#  界面操作：空格 = 开始一次测量；Esc = 停止。一次测量完成后界面显示
#  “测量完成”，再按一次空格即开始第二次；两个结果包分别落在
#  simulation/output/measurement_<任务号>/。
#
#  ⚠ 本脚本**不**启用真实相机、真实转台或真实触发；run_mode=simulation
#     的一致性校验会在启动时拒绝任何混入的真实部件。
#  ⚠ 本脚本不构成实机验收或运动授权。转台发送门禁仍固定为只读。
#
#  用法：
#      scripts/run_simulation.sh              # 配置/构建/生成模型/启动
#      scripts/run_simulation.sh --no-build   # 跳过配置与构建
#      APS_SIM_BUILD_DIR=build-sim scripts/run_simulation.sh
# ============================================================================

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${APS_SIM_BUILD_DIR:-${ROOT}/build-simulation}"
JOBS="${APS_SIM_JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}"
DO_BUILD=1

for arg in "$@"; do
    case "${arg}" in
        --no-build) DO_BUILD=0 ;;
        -h|--help)
            # 只打印文件头注释（第 2–24 行），不把 `set -euo pipefail`
            # 当帮助文本输出；`^# ?` 同时覆盖单空格与无空格的行。
            sed -n '2,24p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "未知参数：${arg}（只接受 --no-build / --help）" >&2
            exit 2
            ;;
    esac
done

cd "${ROOT}"

if [[ "${DO_BUILD}" -eq 1 ]]; then
    # APS_SDK_MODE=OFF：仿真不需要相机 SDK，也避免把本机的 SDK 可用性
    # 混进这条交付路径。BUILD_TESTING=OFF：启动入口只需要两个目标。
    cmake -S "${ROOT}" -B "${BUILD_DIR}" \
          -DCMAKE_BUILD_TYPE=Release \
          -DAPS_SDK_MODE=OFF \
          -DBUILD_TESTING=OFF
    cmake --build "${BUILD_DIR}" \
          --target AircraftPoseSystem gen_synthetic_model \
          -j "${JOBS}"
fi

for exe in AircraftPoseSystem gen_synthetic_model; do
    if [[ ! -x "${BUILD_DIR}/${exe}" ]]; then
        echo "缺少 ${BUILD_DIR}/${exe}；请去掉 --no-build 后重试。" >&2
        exit 1
    fi
done

# 机型库必须每次由同一份几何重新生成。脚本在**启动前**做，而不是交给
# 用户手工执行，是为了让“画面几何”和“PnP 点表”不可能来自不同版本。
"${BUILD_DIR}/gen_synthetic_model" "${ROOT}/simulation/models"

mkdir -p "${ROOT}/simulation/logs" "${ROOT}/simulation/output"

echo
echo "仿真配置：${ROOT}/simulation/config"
echo "结果目录：${ROOT}/simulation/output"
echo "日志目录：${ROOT}/simulation/logs"
echo "操作：空格 = 开始测量（完成后可再按一次开始第二次），Esc = 停止"
echo

# 配置里的相对路径相对**进程当前目录**；上面的 cd "${ROOT}" 是必需的。
exec "${BUILD_DIR}/AircraftPoseSystem" "${ROOT}/simulation/config"
