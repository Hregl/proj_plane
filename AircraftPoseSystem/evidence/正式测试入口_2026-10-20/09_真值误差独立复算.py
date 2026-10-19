#!/usr/bin/env python3
"""独立复算 B 批结果包中的真值误差（不引用工程源码、不使用 OpenCV/numpy）。

判据定义与工程一致（src/app/ApplicationContext.h::rotationErrorDeg）：
  旋转误差 = acos(clamp((trace(R_truth^T · R_est) - 1) / 2, -1, 1)) [deg]
  平移误差 = ||t_est - t_truth||₂ [m]

用法：python3 09_真值误差独立复算.py <result.json>
"""
import json
import math
import sys


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def rotation_error_deg(r_truth, r_est):
    r = mat_mul(transpose(r_truth), r_est)
    c = (r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


def translation_error_m(t_truth, t_est):
    return math.sqrt(sum((e - t) ** 2 for e, t in zip(t_est, t_truth)))


def main():
    if len(sys.argv) != 2:
        print("用法: python3 09_真值误差独立复算.py <result.json>")
        return 2
    with open(sys.argv[1], "r", encoding="utf-8") as f:
        pkg = json.load(f)

    truth = pkg["truth"]["aircraftToShip"]
    est = pkg["aircraftToShip"]
    rot = rotation_error_deg(truth["rotation"], est["rotation"])
    tr = translation_error_m(truth["translation"], est["translation"])

    print(f"任务号            : {pkg['task_id']}")
    print(f"应用终态          : {pkg['state']}")
    print(f"独立复算旋转误差  : {rot:.6f} deg = {rot * 60.0:.3f} 角分")
    print(f"包内记录旋转误差  : {pkg['truth']['error']['rotation_deg']:.6f} deg")
    print(f"旋转差值          : {abs(rot - pkg['truth']['error']['rotation_deg']):.3e} deg")
    print(f"独立复算平移误差  : {tr:.6f} m = {tr * 1000.0:.1f} mm")
    print(f"包内记录平移误差  : {pkg['truth']['error']['translation_m']:.6f} m")
    print(f"平移差值          : {abs(tr - pkg['truth']['error']['translation_m']):.3e} m")
    print(f"重投影误差        : {pkg['reprojection_error']:.6f} px")
    print(f"使用相机          : {pkg['camera_used']}")
    print(f"匹配数/CAD 对应数 : {pkg['statistics']['match_count']}/"
          f"{pkg['statistics']['cad_count']}，inlier_ratio="
          f"{pkg['validation']['inlier_ratio']:.6f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
