#ifndef SAMPLE_LOG_H
#define SAMPLE_LOG_H

#include <stdio.h>

// #define __SAMPLE_LOG_EN__
#ifndef SAMPLE_LOG_TAG
#define SAMPLE_LOG_TAG "COMMON"
#endif

#ifndef SAMPLE_LOG_TAG_WIDTH
#define SAMPLE_LOG_TAG_WIDTH 8
#endif

#define SAMPLE_LOG_PRINT(color, level, fmt, ...)                                             \
    do {                                                                                     \
        printf(color level ":[%-*s] " fmt "\033[0m\n", SAMPLE_LOG_TAG_WIDTH, SAMPLE_LOG_TAG, \
               ##__VA_ARGS__);                                                               \
    } while (0)

#define SAMPLE_LOG_PRINT_ERROR(color, level, fmt, ...)                              \
    do {                                                                            \
        printf(color level ":[%-*s][%s:%d] " fmt "\033[0m\n", SAMPLE_LOG_TAG_WIDTH, \
               SAMPLE_LOG_TAG, __func__, __LINE__, ##__VA_ARGS__);                  \
    } while (0)

/* Development diagnostics: enabled only when __SAMPLE_LOG_EN__ is defined. */
#ifdef __SAMPLE_LOG_EN__
#define ALOGD(fmt, ...) SAMPLE_LOG_PRINT("\033[1;30;37m", "DEBUG ", fmt, ##__VA_ARGS__)
#define ALOGI(fmt, ...) SAMPLE_LOG_PRINT("\033[1;30;32m", "INFO  ", fmt, ##__VA_ARGS__)
#else
#define ALOGD(fmt, ...) \
    do {                \
    } while (0)
#define ALOGI(fmt, ...) \
    do {                \
    } while (0)
#endif

/* Always-on runtime logs. */
#define ALOGW(fmt, ...) SAMPLE_LOG_PRINT("\033[1;30;33m", "WARN  ", fmt, ##__VA_ARGS__)
#define ALOGE(fmt, ...) SAMPLE_LOG_PRINT_ERROR("\033[1;30;31m", "ERROR ", fmt, ##__VA_ARGS__)
#define ALOGN(fmt, ...) SAMPLE_LOG_PRINT("\033[1;30;36m", "NOTICE", fmt, ##__VA_ARGS__)

#endif
