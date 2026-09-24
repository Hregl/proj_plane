# third_party —— 第三方 SDK 台账

> **本文件必须随代码版本化。** 本目录下的 `imvsdk/` **不**进入版本管理
> （见根目录 `.gitignore` 的"第三方设备 SDK"节）。
>
> 依据：评审 2026-09-23 裁决 ——
> > 理由：SDK 通常涉及：**授权；厂商文件；平台绑定**。
> > 建议：`third_party/imvsdk/` 进入 `.gitignore`；
> > 同时保留 `third_party/README.md`，记录**SDK 名称 / 版本 / 来源 / 安装方式 / 校验信息**。
>
> 落位方式依据 O-19：**解包**，不 `make install`（避免污染系统、避免 Qt 混入）。

---

## 1 ImvSdk（华睿 / Huaray 工业相机 SDK）

### 1.1 基本信息

| 项 | 值 |
|---|---|
| **SDK 名称** | 机器视觉工业相机客户端 **MVviewer**（内含 IMV SDK：`libMVSDK` + `IMVApi.h`） |
| **厂商** | ZheJiang Huray Technology Co., Ltd.（浙江华睿科技） |
| **客户端版本** | `Ver2.7.0.1` |
| **构建号** | `Build20260709`（包内文件时间戳 2026-07-09） |
| **SDK 库版本** | `libMVSDK.so.2.7.0.1.422704`（版本 2.7.0.1，build 422704） |
| **手册版本** | 《Linux SDK 使用手册》Ver1.0（配套 SDK 2.4.1，**早于本包**，仅供参照） |
| **目标机型** | 华睿 **A7A20MU201**（ENG-08 §6 Sprint 2） |
| **适用平台** | x86_64 / i686 Linux；glibc ≥ 2.12；glibc 支持的**内核 2.6.32 ~ 6.2.0**（手册 §1.2） |
| **本机平台** | UOS x86_64，GCC 12.3.0，**内核 6.6.0-amd64-desktop** ⚠ 见 §4 |

### 1.2 来源

| 项 | 值 |
|---|---|
| **位置** | `<仓库根>/sdk/机器视觉工业相机客户端MVviewer+Ver2.7.0.1（Linux+x86）/`（`sdk/` 与 `AircraftPoseSystem/` 同级，**不入版本管理**） |
| **安装包文件名** | `MVviewer_Ver2.7.0.1_Linux_x86_Build20260709.run` |
| **包类型** | **Makeself 2.1.5** 自解压归档（POSIX shell 头 + gzip 压缩的 tar 载荷） |
| **文件大小** | `84250456` 字节（约 81 MB） |
| **整文件 MD5** | `6e8f1b7348e4a25a3d7a3a57c35eab12` |
| **归档声明的载荷 MD5** | `3ca4a867ec39fb4d682b46ad87a07cf1`（`MD5=` 字段） |
| **归档 CRC** | `1044149578`（`CRCsum=` 字段） |
| **归档条目数** | 831 |
| **随包文档** | 同目录另有 4 份 docx：`Linux环境下安装MVviewer方法` / `Linux基础操作文档` / `linux简易操作手册` / `Linux_SDK使用手册_Ver1.0` |

⚠ **校验方式的更正**：归档内 `MD5=` 字段是**压缩载荷**的校验和，**不是整文件 MD5**。
因此"用 `md5sum 整个 .run` 与包内声明比对"是一个**假校验**（必然不相等，实测
`6e8f1b…` vs `3ca4a8…`）。正确的完整性校验是归档自带的：

```bash
sh MVviewer_..._Build20260709.run --check
# → Verifying archive integrity... MD5 checksums are OK. All good.   （2026-09-23 实测通过）
```

### 1.3 落位方式（**解包，不安装**）

厂商的 `bin/common/install.sh` 会：**要求 root**、拷到 `/opt/HuarayTech/MVviewer`、
改写 `/etc/rc.local` 以便开机加载内核驱动、并检查内核版本区间。
按 O-19 裁决，**不执行它** —— 改为**只解包**：

```bash
cd AircraftPoseSystem
sh "../sdk/机器视觉工业相机客户端MVviewer+Ver2.7.0.1（Linux+x86）/MVviewer_Ver2.7.0.1_Linux_x86_Build20260709.run" \
   --noexec --nochown \
   --target third_party/imvsdk \
   --exclude='./lib/m64x86/Qt' \
   --exclude='./bin/common/Skins' \
   ./include ./lib/m64x86 ./module ./share/C ./license ./doc ./bin/common ./install.sh
```

**为什么是选择性解包（644 / 831 条目，27 MB vs 全量约 216 MB）**：
包内 410 条是 MVViewer **图形客户端**（`bin/<arch>/` 的图标/字体/Qt platform 插件、
`Skins/`），219 条是例程。本工程要的是 SDK 本身，**且全量解包会把两套 Qt 拖进工程**
（`lib/m64x86/Qt` = Qt **5.6.3**，`lib/m32x86/Qt` = Qt **5.5.1**），
正是 O-19 要避免的污染。故明确**未取**：`bin/m32x86`、`bin/m64x86`、`lib/m32x86`、
`lib/m64x86/Qt`、`share/QT`、`share/Python`、`bin/common/Skins`。

⚠ **`--noexec` 是本节的硬性要求**；`--nochown` 避免在共享机上改属主。
⚠ 直接用 `--target` 解包会命中 Makeself 的 xterm 分支而报
`exec: -title: not found`（实测）；**改用 `--tar` 通道**（本节即为 `--tar` 形式）。

### 1.4 解包后的目录（本机实测）

```
third_party/imvsdk/                    27 MB
├ include/                             ← 4 个头文件
│   ├ IMVApi.h                         （C 接口主头，84 个 API）
│   ├ IMVDefines.h                     （错误码 / 常量）
│   ├ IMVFGApi.h   IMVFGDefines.h
├ lib/m64x86/                          ← 64 位库
│   ├ libMVSDK.so.2.7.0.1.422704       （主库，5.9 MB，**无 SONAME、无 .so 软链**）
│   ├ libImageConvert.so  libImageSave.so  libTinyXml.so  libspc.so
│   ├ libcompress_decode.so  liblog4cpp.so  libusb-1.0.so  libAuth.so
│   ├ libiImageProcessing64.so  libquazip.so  libVideoRender.so
│   ├ libMVSDKGuiQt.so                 （GUI 用，本工程**不取用**）
│   └ GenICam/bin/Linux64_x64/         （7 个 lib*_gcc421_v3_0.so，libMVSDK 的硬依赖）
├ module/common/                       ← 驱动（GigE 内核模块 + RayDriverApi 源码）
├ share/C/IMV/                         ← 官方 C 例程（20+ 个，含 Makefile）
├ bin/common/                          ← 厂商脚本（install/uninstall/run/set_usb_stack/…）
├ license/                             ← 第三方许可
└ install.sh                           ← 厂商安装器（**未执行**）
```

### 1.5 运行期要求（详见 `cmake/FindImvSdk.cmake` 与 011-A0.1 核验报告）

| 项 | 结论 |
|---|---|
| **链接期** | `include/` + `-L<runtime> -lMVSDK`。⚠ 不能写 `find_library(NAMES MVSDK)` —— 包内**没有** `libMVSDK.so`，且主库**无 SONAME** |
| **运行期库** | 由 CMake 在**构建树**生成 `imvsdk_runtime/`（软链齐全部依赖 + 修正 3 个 SONAME/文件名不符者）。实测 `ldd` 0 个 `not found`、`dlopen` 成功 |
| **U3V/USB 相机** | ⚠ 需 `echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb`（**需 root**，见 `bin/common/set_usb_stack.sh`）。不改会导致缓冲不足/丢帧 |
| **GenICam 缓存** | `GENICAM_CACHE_V3_0` 可选；厂商脚本只设 `LD_LIBRARY_PATH`，故仅在有缓存告警时设 |
| **日志** | `/var/log/MVSDK/<程序名>`，配置文件 `SDKLOG_default.properties` 自动生成；**无权限时需 sudo** |

---

## 2 本目录的维护约定

1. `imvsdk/` **不入版本管理**；换机器按 §1.3 的命令重新解包即可。
2. **SDK 版本或来源变更时，必须更新本文件的 §1.1 / §1.2 表格**，并重跑
   `--check` 与核验报告的 §6 检查项。台账与实物不符比没有台账更危险。
3. `cmake/FindImvSdk.cmake` 内记录的**实测名**（库名、平台子目录、SONAME 修正表）
   是与本文件配套的：**换 SDK 版本时两者要一起改**，否则会出现
   "配置能过、打开相机时才失败"的失效形态。
