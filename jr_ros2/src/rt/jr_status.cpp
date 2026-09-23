/**
 * @file    jr_status.cpp
 * @brief   状态码/建议文本与 Result 实现
 */

#include "jr_ros2/jr_status.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace jr {

const char *to_string(Status s) noexcept
{
    switch (s) {
    case Status::kOk:              return "ok";
    case Status::kInvalidArgument: return "invalid-argument";
    case Status::kInvalidState:    return "invalid-state";
    case Status::kTimeout:         return "timeout";
    case Status::kTransport:       return "transport";
    case Status::kProtocol:        return "protocol";
    case Status::kNotFound:        return "not-found";
    case Status::kNotSupported:    return "not-supported";
    case Status::kNoMemory:        return "no-memory";
    case Status::kNotCalibrated:   return "not-calibrated";
    case Status::kLocked:          return "locked";
    case Status::kIoError:         return "io-error";
    case Status::kUnverified:      return "unverified";
    case Status::kInternal:        return "internal";
    }
    return "unknown";
}

const char *to_string(Advice a) noexcept
{
    switch (a) {
    case Advice::kNone:                      return "none";
    case Advice::kRetryFaultReset:           return "retry-fault-reset";
    case Advice::kNeedsDeviceReset:          return "needs-device-reset";
    case Advice::kCheckBusTermination:       return "check-bus-termination";
    case Advice::kCheckBusConfig:            return "check-bus-config";
    case Advice::kReduceRateOrRaiseWatchdog: return "reduce-rate-or-raise-watchdog";
    case Advice::kCheckTemperature:          return "check-temperature";
    case Advice::kCheckSupplyVoltage:        return "check-supply-voltage";
    case Advice::kCalibrationRequired:       return "calibration-required";
    case Advice::kEnsureSingleMaster:        return "ensure-single-master";
    case Advice::kCheckRtPermissions:        return "check-rt-permissions";
    case Advice::kFixBusPlanning:            return "fix-bus-planning";
    }
    return "unknown";
}

void Result::set(Status s, Advice a, const char *fmt, ...) noexcept
{
    status = s;
    advice = a;

    if (fmt == nullptr) {
        message[0] = '\0';
        truncated = false;
        return;
    }

    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(message, sizeof(message), fmt, ap);
    va_end(ap);

    /* vsnprintf 返回"若缓冲足够大会写多少"，据此显式标记截断。 */
    truncated = (n < 0) || (static_cast<std::size_t>(n) >= sizeof(message));
    message[sizeof(message) - 1u] = '\0';
}

void Result::set_from(const Result &other) noexcept
{
    status = other.status;
    advice = other.advice;
    truncated = other.truncated;
    std::memcpy(message, other.message, sizeof(message));
}

}  // namespace jr
