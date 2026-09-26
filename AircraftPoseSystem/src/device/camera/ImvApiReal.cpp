// ============================================================================
//  src/device/camera/ImvApiReal.cpp
//
//  依据：SYS-06 §7、ENG-08 §3 / §11、ENG-09 V2.3 §2.5 / §2.6
//
//  作用：`IImvApi` 的**真实 SDK 实现** —— 本工程中**唯一**直接引用
//  `IMV_*` 符号的文件。
//
//  ⚠ 本文件整体包在 `#if APS_HAVE_IMVSDK` 里，**这是"S有没有 SDK 头文件"
//  这一物理事实决定的**，不是可选的整理手法：无 SDK 时 `IMVApi.h` 不存在，
//  任何引用都会编译失败。`APS_HAVE_IMVSDK` 由
//  `src/device/camera/CMakeLists.txt:67-73` 从 Dependencies.cmake 的查找结果
//  导出（**总是**定义 0/1，故 `#if` 能区分"没 SDK"与"宏名拼错"）。
//
//  ⚠ 与既有注释的关系（2026-09-26 更正）：`ImvCameraBackend.h` 旧文写
//  "当前构建中 `APS_HAVE_IMVSDK` 未定义（SDK 未安装）"。该句**已失效**：
//  本机 `third_party/imvsdk/` 实际存在，`build/` 里的 `APS_HAVE_IMVSDK` 是 1
//  （构建期可见 `#pragma message` 即由此而来）。故此处的规则改为如实陈述：
//  **找到则编译真实实现，未找到则 `makeRealImvApi()` 返回 nullptr**。
//
//  ⚠ 本文件**不含任何业务判断**：不归类错误码、不校验帧、不算长度。
//  那些都是 `ImvCameraBackend` 的职责（§2.2 映射表与 §3.2 的校验顺序），
//  在此处再写一份，两处迟早漂移 —— 而漂移之后"SDK 原码"就不可考了。
// ============================================================================

#include "device/camera/IImvApi.h"

#if APS_HAVE_IMVSDK

#include <cstring>
#include <unordered_map>

#include "IMVApi.h"

namespace aircraft
{
namespace device
{
namespace
{

/// 把 SDK 的定长 `char[N]` 拷成 `std::string`。
///
/// ⚠ 用 `strnlen` 而不是 `strlen`：SDK 的字段注释保证"不超过 255"，
/// 但那是**注释**而非强制，若某台设备回填满了 256 字节而无终止符，
/// `strlen` 会越界读 —— 这是本工程唯一一处直接读设备回报字符串的地方，
/// 越界读一个未映射页就是段错误，且现场很难复现。
std::string toString(const char* field, std::size_t capacity)
{
    return std::string(field, ::strnlen(field, capacity));
}

/// SDK 的 `IMV_String` → `std::string`。
std::string toString(const IMV_String& s)
{
    return toString(s.str, IMV_MAX_STRING_LENTH);
}

/// 真实 SDK 访问层。
class ImvApiReal : public IImvApi
{
public:
    int enumDevices(std::vector<ImvDeviceEntry>& out) override
    {
        IMV_DeviceList list;
        ::memset(&list, 0, sizeof(list));

        const int ret = IMV_EnumDevices(&list, interfaceTypeAll);
        if (ret != IMV_OK)
        {
            return ret;
        }

        // ⚠ `list.pDevInfo` 是 **SDK 内部缓存**（头文件原文 "cached within
        // the SDK"，IMVDefines.h:540-544），**且没有释放接口** ——
        // 因此：① 不得 free；② 必须在本次调用内把用到的字段**拷出来**，
        // 因为下一处 `IMV_EnumDevices` 会覆写同一块缓存。
        out.clear();
        out.reserve(list.nDevNum);
        for (unsigned int i = 0; i < list.nDevNum; ++i)
        {
            const IMV_DeviceInfo& d = list.pDevInfo[i];

            ImvDeviceEntry e;
            e.cameraKey     = toString(d.cameraKey, IMV_MAX_STRING_LENTH);
            e.cameraName    = toString(d.cameraName, IMV_MAX_STRING_LENTH);
            e.serialNumber  = toString(d.serialNumber, IMV_MAX_STRING_LENTH);
            e.vendorName    = toString(d.vendorName, IMV_MAX_STRING_LENTH);
            e.modelName     = toString(d.modelName, IMV_MAX_STRING_LENTH);
            e.interfaceType = static_cast<int32_t>(d.nInterfaceType);
            out.push_back(std::move(e));
        }
        return IMV_OK;
    }

    int createHandle(void*& handle, ImvHandleMode mode, void* identifier) override
    {
        // ⚠ 只实现 ByIndex。SDK 的枚举常量与我们的枚举**取值巧合相同**
        // （modeByIndex == 0），但**不依赖这个巧合**：显式翻译，
        // 使 SDK 侧将来调整取值时这里不会静默错位。
        IMV_ECreateHandleMode sdkMode;
        switch (mode)
        {
        case ImvHandleMode::ByIndex:
            sdkMode = modeByIndex;
            break;
        default:
            return IMV_INVALID_PARAM;
        }

        IMV_HANDLE h = nullptr;
        const int  ret = IMV_CreateHandle(&h, sdkMode, identifier);
        if (ret != IMV_OK)
        {
            return ret;
        }

        handle = static_cast<void*>(h);
        return IMV_OK;
    }

    int open(void* handle) override
    {
        return IMV_Open(static_cast<IMV_HANDLE>(handle));
    }

    int close(void* handle) override
    {
        return IMV_Close(static_cast<IMV_HANDLE>(handle));
    }

    int destroyHandle(void* handle) override
    {
        return IMV_DestroyHandle(static_cast<IMV_HANDLE>(handle));
    }

    int getDeviceInfo(void* handle, ImvDeviceEntry& out) override
    {
        IMV_DeviceInfo d;
        ::memset(&d, 0, sizeof(d));

        const int ret = IMV_GetDeviceInfo(static_cast<IMV_HANDLE>(handle), &d);
        if (ret != IMV_OK)
        {
            return ret;
        }

        out.cameraKey     = toString(d.cameraKey, IMV_MAX_STRING_LENTH);
        out.cameraName    = toString(d.cameraName, IMV_MAX_STRING_LENTH);
        out.serialNumber  = toString(d.serialNumber, IMV_MAX_STRING_LENTH);
        out.vendorName    = toString(d.vendorName, IMV_MAX_STRING_LENTH);
        out.modelName     = toString(d.modelName, IMV_MAX_STRING_LENTH);
        out.interfaceType = static_cast<int32_t>(d.nInterfaceType);
        return IMV_OK;
    }

    int setEnumFeatureSymbol(void* handle, const char* feature,
                             const char* symbol) override
    {
        return IMV_SetEnumFeatureSymbol(static_cast<IMV_HANDLE>(handle), feature,
                                       symbol);
    }

    int getEnumFeatureSymbol(void* handle, const char* feature,
                             std::string& out) override
    {
        IMV_String s;
        ::memset(&s, 0, sizeof(s));

        const int ret =
            IMV_GetEnumFeatureSymbol(static_cast<IMV_HANDLE>(handle), feature, &s);
        if (ret != IMV_OK)
        {
            return ret;
        }

        out = toString(s);
        return IMV_OK;
    }

    int setIntFeatureValue(void* handle, const char* feature,
                           int64_t value) override
    {
        return IMV_SetIntFeatureValue(static_cast<IMV_HANDLE>(handle), feature,
                                      value);
    }

    int getIntFeatureValue(void* handle, const char* feature,
                           int64_t& out) override
    {
        return IMV_GetIntFeatureValue(static_cast<IMV_HANDLE>(handle), feature,
                                      &out);
    }

    int setDoubleFeatureValue(void* handle, const char* feature,
                              double value) override
    {
        return IMV_SetDoubleFeatureValue(static_cast<IMV_HANDLE>(handle), feature,
                                         value);
    }

    int getDoubleFeatureValue(void* handle, const char* feature,
                              double& out) override
    {
        return IMV_GetDoubleFeatureValue(static_cast<IMV_HANDLE>(handle), feature,
                                         &out);
    }

    int executeCommandFeature(void* handle, const char* feature) override
    {
        return IMV_ExecuteCommandFeature(static_cast<IMV_HANDLE>(handle), feature);
    }

    int startGrabbing(void* handle) override
    {
        return IMV_StartGrabbing(static_cast<IMV_HANDLE>(handle));
    }

    int stopGrabbing(void* handle) override
    {
        return IMV_StopGrabbing(static_cast<IMV_HANDLE>(handle));
    }

    int getFrame(void* handle, ImvFrameView& out, unsigned int timeoutMs) override
    {
        // ⚠ 帧结构体必须在本对象里**存活到 `releaseFrame()`**：
        // SDK 的 `IMV_ReleaseFrame` 收的是同一个 `IMV_Frame*`
        // （它内部要据此归还 SDK 自己的缓存），故不能是局部变量。
        // 用句柄做键而不是"当前帧"单成员：`ICameraBackend` 虽然约定
        // 设备线程独占，但按句柄索引不额外付出代价，且消除了
        // "两个句柄交错取帧"这一**在测试里会出现**（替身同时驱动多路）
        // 场景下的静默错配。
        IMV_Frame& frame = frames_[handle];
        ::memset(&frame, 0, sizeof(frame));

        const int ret =
            IMV_GetFrame(static_cast<IMV_HANDLE>(handle), &frame, timeoutMs);
        if (ret != IMV_OK)
        {
            // ⚠ 失败时**不填** `out` 的任何字段（含"清零"）。
            // 头文件未保证失败时 `frame.frameInfo` 的输出字段有效，
            // 把可能无效的内容当成事实带出去，正是 §2.2 禁止
            // "失败时去读 frameInfo 交叉判断"的同一件事。
            return ret;
        }

        out.data          = frame.pData;
        out.blockId       = frame.frameInfo.blockId;
        out.status        = frame.frameInfo.status;
        out.width         = frame.frameInfo.width;
        out.height        = frame.frameInfo.height;
        out.size          = frame.frameInfo.size;
        out.pixelFormat   = static_cast<int32_t>(frame.frameInfo.pixelFormat);
        out.timeStamp     = frame.frameInfo.timeStamp;
        out.paddingX      = frame.frameInfo.paddingX;
        out.paddingY      = frame.frameInfo.paddingY;
        out.recvFrameTime = frame.frameInfo.recvFrameTime;
        return IMV_OK;
    }

    int releaseFrame(void* handle) override
    {
        auto it = frames_.find(handle);
        if (it == frames_.end())
        {
            // 没有取得过的帧却要求归还：本地时序错误，未调用 SDK。
            return IMV_INVALID_PARAM;
        }

        const int ret =
            IMV_ReleaseFrame(static_cast<IMV_HANDLE>(handle), &it->second);
        frames_.erase(it);
        return ret;
    }

private:
    std::unordered_map<void*, IMV_Frame> frames_;
};

}  // namespace

std::shared_ptr<IImvApi> makeRealImvApi()
{
    return std::make_shared<ImvApiReal>();
}

}  // namespace device
}  // namespace aircraft

#else  // !APS_HAVE_IMVSDK

namespace aircraft
{
namespace device
{

std::shared_ptr<IImvApi> makeRealImvApi()
{
    // 未接入 SDK：返回空指针，使后端走"诚实桩"路径并给出可操作提示
    // （"请用 -DIMV_SDK_ROOT=… 重新配置，或使用虚拟相机"）。
    // ⚠ 不返回一个"每步都失败"的假对象：那会让"这台机器上没有 SDK"
    //    与"S某个特性调用失败"在日志里看起来是同一类事件。
    return nullptr;
}

}  // namespace device
}  // namespace aircraft

#endif  // APS_HAVE_IMVSDK
