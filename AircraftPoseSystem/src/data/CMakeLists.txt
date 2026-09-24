# ============================================================================
#  src/data/CMakeLists.txt  →  libdata
#
#  依据：ENG-03 §12.1（生成 libdata，依赖：无）
#        ENG-01 §5（data 模块文件清单）
#
#  依赖：**无模块依赖** + OpenCV（见下方说明）
#
#  为什么 data 必须无模块依赖（ENG-01 §18 第三条"Data 包含业务"的镜像约束）：
#  一旦 data 引入任何**本项目模块**的依赖，依赖图立刻出现环——因为所有人都
#  依赖 data。这也解释了裁决 C-07：CameraRole / Transform / CoordinateFrame
#  等类型必须下沉到本模块，而不是留在 optical/。若留在 optical/，则 data 中的
#  ImageFrame、DetectionResult、MeasurementCandidate 反过来需要包含 optical
#  的头文件，data 就不再是零依赖层，algorithm（只依赖 data + OpenCV）也将
#  不可达。见 ENG-09 §3.2。
#
#  ⚠ 关于引擎 OpenCV —— 与 ENG-03 §12.1 的"依赖：无"表面冲突，实际不冲突：
#
#  ENG-09 §5.1 冻结的 Transform 是
#      struct Transform { cv::Matx33d rotation; cv::Vec3d translation; };
#  同类还有 CameraCalibration（cv::Mat 内参/畸变）、ImageFrame（cv::Mat image）、
#  DetectionResult（cv::Rect）、FeatureCorrespondence（cv::Point3f/2f）、
#  TargetScaleEstimate、ModelPoint3D 等。
#  这些是 data 的**公开头文件**，任何包含它们的编译单元都需要 OpenCV 头文件
#  与运行期库。因此 libdata 必须链接 OpenCV，且必须是 PUBLIC——
#  写成 PRIVATE 则消费者在链自己的目标时找不到 OpenCV 符号，链接期报
#  undefined reference to cv::Mat::Mat()，而报错位置指向消费者，与真实原因
#  相距甚远。
#
#  §12.1 的"依赖：无"应读作"**无本项目模块依赖**"：
#    · 它描述的是 ENG-01 §17 依赖链中的位置（最底层，无人可依赖）；
#    · 第三方库不在该依赖链的讨论范围内——§12.5 给 algorithm 写的是
#      "data + OpenCV"，可见该列表本就混列模块与第三方库，§12.1 只是省略了
#      OpenCV（当时的关注点是模块分层，不是引擎依赖）。
#
#  结论：本项**不改变分层**（OpenCV 是第三方引擎，不参与模块依赖图，
#  不产生环），因此 ENG-01 §17 的冻结依赖链与 §18 的禁止依赖均未被破坏。
#
#  本次由编译探针实测发现（把含 cv::Mat 的头文件放进 data 并编译 optical
#  消费者时立刻失败）。ENG-03 §12.1 建议补一行 OpenCV，与 §12.2~§12.8
#  的写法对齐。
# ============================================================================

aps_add_module_library(data
    DEPENDS
        aps::opencv
)
