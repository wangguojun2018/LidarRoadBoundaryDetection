/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-09-03 18:22:43
 */

/**
 * @file
 */

#ifndef MODULES_COMMON_LOG_H_
#define MODULES_COMMON_LOG_H_

#include "glog/logging.h"
#include "glog/raw_logging.h"

#define ADEBUG VLOG(4) << "[DEBUG] "
#define AINFO LOG(INFO)
#define AWARN LOG(WARNING)
#define AERROR LOG(ERROR)
#define AFATAL LOG(FATAL)

// LOG_IF
#define AINFO_IF(cond) LOG_IF(INFO, cond)
#define AERROR_IF(cond) LOG_IF(ERROR, cond)
#define ACHECK(cond) CHECK(cond)

// LOG_EVERY_N
#define AINFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define AWARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define AERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#define RETURN_IF_NULL(ptr)               \
    if (ptr == nullptr) {                 \
        AWARN << #ptr << " is nullptr.";  \
        return;                           \
    }

#define RETURN_VAL_IF_NULL(ptr, val)      \
    if (ptr == nullptr) {                 \
        AWARN << #ptr << " is nullptr.";  \
        return val;                       \
    }

#define RETURN_IF(condition)                   \
    if (condition) {                           \
        AWARN << #condition << " is not met."; \
        return;                                \
    }

#define RETURN_VAL_IF(condition, val)          \
    if (condition) {                           \
        AWARN << #condition << " is not met."; \
        return val;                            \
    }

#endif  // MODULES_COMMON_LOG_H_
