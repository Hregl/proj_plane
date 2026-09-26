// ============================================================================
//  src/data/ErrorInfo.cpp
//
//  依据：ENG-09 §5.27（错误码冻结表）、ENG-01 §5 与 ENG-03 §12.1 的调和
//
//  data 模块的**唯一编译单元**。存在的理由见 ErrorInfo.h 文件头：
//  ENG-01 §5 规定 data 为纯头文件模块，而 ENG-03 §12.1 要求产出
//  libdata（静态库）。静态库必须至少有一个目标文件，否则归档为空、
//  链接时表现为"找不到符号"这类难以定位的错误。
//
//  本文件只做一件事：把冻结的错误码映射为稳定名称。
//  不引入业务逻辑，不访问外部资源，不做 I/O ——
//  因此不违反"data 层只有值类型与共用枚举"的约束。
//
//  使用者：infrastructure/logger 在写故障日志时调用，
//  使日志中出现 "ErrTaskTimeout(9001)" 而不是裸数字 9001 ——
//  离线排查时可以直接按名字检索，不必先查表。
// ============================================================================

#include "data/ErrorInfo.h"

namespace aircraft
{
namespace data
{

const char* errorCodeName(int code)
{
    switch (code)
    {
    case 0:
        return "OK";

    // ---- 相机设备段 ----
    case kErrCameraInsufficient:
        return "ErrCameraInsufficient";
    case kErrCameraDegraded:
        return "ErrCameraDegraded";
    case kErrGrabTimeout:
        return "ErrGrabTimeout";
    case kErrCameraNotStarted:
        return "ErrCameraNotStarted";
    case kErrCameraSdkError:
        return "ErrCameraSdkError";
    case kErrGrabContract:
        return "ErrGrabContract";
    case kErrCameraUnsupported:
        return "ErrCameraUnsupported";

    // ---- 转台设备段 ----
    case kErrTurntableComm:
        return "ErrTurntableComm";
    case kErrTurntableOverTravel:
        return "ErrTurntableOverTravel";
    case kErrAlignRetryExhausted:
        return "ErrAlignRetryExhausted";

    // ---- 触发 / 同步段 ----
    case kErrTriggerDegraded:
        return "ErrTriggerDegraded";
    case kErrSyncOutOfTolerance:
        return "ErrSyncOutOfTolerance";

    // ---- 标定段（裁决 C-006）----
    case kErrCalibrationMissing:
        return "ErrCalibrationMissing";

    // ---- 模型与特征库段（裁决 C-006）----
    case kErrModelMissing:
        return "ErrModelMissing";

    // ---- 算法段 ----
    case kErrPnpRetryExhausted:
        return "ErrPnpRetryExhausted";

    // ---- 系统级 ----
    case kErrTaskTimeout:
        return "ErrTaskTimeout";
    case kErrRollbackExhausted:
        return "ErrRollbackExhausted";
    case kErrManualCancel:
        return "ErrManualCancel";
    case kErrStateFailure:
        return "ErrStateFailure";
    case kErrSystemConfig:
        return "ErrSystemConfig";

    default:
        // 有意不区分"段内未登记"与"完全越界"：两者都说明产生了
        // ENG-09 §5.27 表外的码，都属于必须修复的缺陷。
        // 日志中配合 code 数值即可定位。
        return "UNREGISTERED";
    }
}

}  // namespace data
}  // namespace aircraft
