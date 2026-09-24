#pragma once

// ============================================================================
//  src/data/CameraCalibration.h
//
//  依据：ENG-09 §4.1、§5.2（类型定义冻结）、裁决 C-02
//
//  裁决 C-02：SYS-11 §4.1 缺 imageWidth / imageHeight，
//  **以 SYS-05 §4.2 为准，保留**。理由：像素偏差归一化（TargetOffset 的
//  阈值判据）与 PnP 都必须在归一化像平面或像素平面上工作，缺图像尺寸时
//  "相对图像中心多少像素算居中"无法判定。
//
//  单位提醒（ENG-09 §2.3）：本结构体中的平移由 Transform 承担，单位为米；
//  而 cameraMatrix 的单位是像素。**同一结构体内两种单位并存**，这是
//  针孔模型的固有形态（内参在像平面，外参在物理空间），不是笔误。
// ============================================================================

#include "data/Transform.h"

namespace aircraft
{
namespace data
{

/// 单相机的内参 + 该相机到光机刚体的外参。
struct CameraCalibration
{
    /// 3x3 CV_64F 内参矩阵 [[fx,0,cx],[0,fy,cy],[0,0,1]]，单位 pixel。
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

    /// 1xN CV_64F 畸变系数（k1,k2,p1,p2[,k3]）。默认空 = 无畸变。
    cv::Mat distortion;

    /// Camera → OpticalRig。方向语义见 ENG-09 §2.1（本字段方向经核对正确，
    /// 未受 §2.1 的字段名修正影响）。
    Transform cameraToRig;

    /// 图像宽，单位 pixel。SYS-11 的标定产出中曾缺失，裁决 C-02 保留。
    int imageWidth = 0;

    /// 图像高，单位 pixel。
    int imageHeight = 0;
};

}  // namespace data
}  // namespace aircraft
