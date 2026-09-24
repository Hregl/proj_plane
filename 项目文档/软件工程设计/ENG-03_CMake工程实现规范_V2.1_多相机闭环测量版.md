# AircraftPoseSystem CMake工程实现规范 V2.1（多相机闭环测量版）

## 1 文档目的

本文档定义AircraftPoseSystem V2.1的软件构建系统设计。

V2.1相比V2.0新增：

-   OpticalRig模块；
-   Turntable设备模块；
-   Trigger设备模块；
-   MeasurementSelector算法模块；
-   TargetModel模块；
-   PoseValidation模块。

目标：

建立：

-   模块化；
-   可维护；
-   可测试；
-   可离线部署；

的CMake工程体系。

------------------------------------------------------------------------

# 2 CMake设计原则

## 2.1 模块独立编译

每个功能模块独立生成Library。

结构：

    data

    ↓

    optical

    ↓

    device

    ↓

    preview

    ↓

    algorithm

    ↓

    application

    ↓

    ui

    ↓

    app

------------------------------------------------------------------------

## 2.2 单向依赖

冻结：

    app

    ↓

    ui

    ↓

    application

    ↓

    preview / optical

    ↓

    device / algorithm

    ↓

    infrastructure

    ↓

    data

禁止：

-   data依赖Qt；
-   algorithm依赖device SDK；
-   UI直接依赖硬件。

------------------------------------------------------------------------

# 3 工程CMake结构

    AircraftPoseSystem/

    ├── CMakeLists.txt

    ├── cmake/

    ├── src/

    │
    ├── data/

    ├── optical/

    ├── device/

    ├── preview/

    ├── algorithm/

    ├── application/

    ├── infrastructure/

    ├── ui/

    └── app/


    └── tests/

------------------------------------------------------------------------

# 4 根目录CMake

文件：

    CMakeLists.txt

职责：

-   项目定义；
-   编译选项；
-   子模块加载。

示例：

``` cmake
cmake_minimum_required(VERSION 3.16)

project(
AircraftPoseSystem
VERSION 2.1
LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(src)

add_subdirectory(tests)
```

------------------------------------------------------------------------

# 5 CMake配置目录

目录：

    cmake/

包含：

    BuildOptions.cmake

    CompilerOptions.cmake

    Dependencies.cmake

    FindImvSdk.cmake

    FindTurntableSdk.cmake

    InstallRules.cmake

------------------------------------------------------------------------

# 6 编译选项

## BUILD_TESTING

测试开关：

``` cmake
option(
BUILD_TESTING
"Build tests"
ON
)
```

------------------------------------------------------------------------

## ENABLE_GPU

GPU加速：

``` cmake
option(
ENABLE_GPU
"Enable GPU acceleration"
OFF
)
```

------------------------------------------------------------------------

## ENABLE_ASAN

内存检查：

``` cmake
option(
ENABLE_ASAN
"Enable Address Sanitizer"
OFF
)
```

------------------------------------------------------------------------

## ENABLE_TSAN

线程检查：

``` cmake
option(
ENABLE_TSAN
"Enable Thread Sanitizer"
OFF
)
```

------------------------------------------------------------------------

# 7 第三方依赖

统一管理：

    Dependencies.cmake

包括：

-   Qt5/Qt6 Widgets；
-   OpenCV；
-   ImvSDK；
-   GoogleTest；
-   转台SDK。

------------------------------------------------------------------------

# 8 Qt配置

用途：

-   MainWindow；
-   ImageViewer；
-   Panel。

配置：

``` cmake
find_package(
Qt5
COMPONENTS
Widgets
REQUIRED
)
```

链接：

``` cmake
target_link_libraries(
ui
PRIVATE
Qt5::Widgets
)
```

------------------------------------------------------------------------

# 9 OpenCV配置

用途：

-   图像处理；
-   SIFT；
-   PnP；
-   坐标计算。

配置：

``` cmake
find_package(OpenCV REQUIRED)
```

------------------------------------------------------------------------

# 10 ImvSDK配置

文件：

    FindImvSdk.cmake

负责：

-   include路径；
-   library路径；
-   runtime路径。

使用：

``` cmake
find_package(ImvSdk REQUIRED)
```

------------------------------------------------------------------------

# 11 Turntable SDK预留

文件：

    FindTurntableSdk.cmake

用途：

适配未来：

-   Peko SDK；
-   网络协议库；
-   RS485通信库。

设计：

SDK作为可选依赖。

------------------------------------------------------------------------

# 12 src模块设计

------------------------------------------------------------------------

# 12.1 data

生成：

    libdata

依赖：

无。

------------------------------------------------------------------------

# 12.2 optical

生成：

    liboptical

包含：

-   OpticalRig；
-   CalibrationManager；
-   CoordinateTransformer。

依赖：

    data

------------------------------------------------------------------------

# 12.3 device

生成：

    libdevice

包含：

    camera

    turntable

    trigger

依赖：

    data

    optical

    ImvSDK(optional)

    TurntableSDK(optional)

------------------------------------------------------------------------

# 12.4 preview

生成：

    libpreview

包含：

-   PreviewManager；
-   PreviewWorker。

依赖：

    data

    optical

------------------------------------------------------------------------

# 12.5 algorithm

生成：

    libalgorithm

结构：

    algorithm/

    ├── detection

    ├── scale

    ├── selection

    ├── model

    ├── feature

    ├── matcher

    ├── pose

    └── validation

依赖：

    data

    OpenCV

------------------------------------------------------------------------

# 12.6 application

生成：

    libapplication

包含：

-   MeasurementController；
-   AlignmentController；
-   StateMachine。

依赖：

    device

    preview

    optical

    algorithm

------------------------------------------------------------------------

# 12.7 infrastructure

生成：

    libinfrastructure

包含：

-   Logger；
-   ConfigManager；
-   Recorder。

------------------------------------------------------------------------

# 12.8 ui

生成：

    libui

依赖：

    Qt

    application

    preview

------------------------------------------------------------------------

# 12.9 app

生成：

    AircraftPoseSystem

链接：

    ui

    application

    algorithm

    device

    preview

    optical

    infrastructure

    data

------------------------------------------------------------------------

# 13 Algorithm子模块CMake

推荐：

每个子模块独立：

    algorithm/

    ├── detection/CMakeLists.txt

    ├── scale/CMakeLists.txt

    ├── selection/CMakeLists.txt

    ├── model/CMakeLists.txt

    ├── feature/CMakeLists.txt

    ├── matcher/CMakeLists.txt

    ├── pose/CMakeLists.txt

    └── validation/CMakeLists.txt

------------------------------------------------------------------------

# 14 Device子模块CMake

结构：

    device/

    ├── camera

    ├── turntable

    └── trigger

分别管理：

-   Camera SDK；
-   转台SDK；
-   触发接口。

------------------------------------------------------------------------

# 15 测试工程

目录：

    tests/

    ├── unit

    ├── optical

    ├── device

    ├── turntable

    ├── trigger

    ├── preview

    ├── algorithm

    ├── integration

    ├── hardware

    ├── stability

    └── golden

**修正：** 原清单缺 `preview` / `hardware` / `golden` 三项，与 ENG-01 §15、ENG-06 §6/§10 不一致（ENG-06 明确要求 `tests/preview/` 与 `tests/golden/`）。补齐后三处一致。

------------------------------------------------------------------------

启用：

``` cmake
enable_testing()
```

推荐：

GoogleTest。

------------------------------------------------------------------------

# 16 Debug/Release

## Debug

用于：

-   开发；
-   调试；
-   测试。

开启：

-   Debug符号；
-   低优化。

------------------------------------------------------------------------

## Release

用于：

-   部署；
-   性能测试。

开启：

-   编译优化；
-   安装打包。

------------------------------------------------------------------------

# 17 install部署

目标：

    deploy/

    ├── bin

    ├── lib

    ├── config

    ├── models

    └── calibration

------------------------------------------------------------------------

安装：

    cmake --install build

------------------------------------------------------------------------

# 18 编译流程

开发：

``` bash
mkdir build

cd build

cmake ..

make
```

------------------------------------------------------------------------

测试：

``` bash
cmake ..

-DENABLE_ASAN=ON

make

ctest
```

------------------------------------------------------------------------

发布：

``` bash
cmake ..

-DCMAKE_BUILD_TYPE=Release

make install
```

------------------------------------------------------------------------

# 19 版本基线

    v2.1-framework

    工程骨架


    v2.1-device

    硬件抽象


    v2.1-algorithm

    算法链


    v2.1-release

    交付版本

------------------------------------------------------------------------

# 20 构建验收标准

必须满足：

1.  所有模块独立编译；
2.  依赖方向正确；
3.  Debug通过；
4.  Release通过；
5.  测试可执行；
6.  支持离线部署；
7.  支持硬件替换。
