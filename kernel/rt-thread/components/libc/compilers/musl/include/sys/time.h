/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-09-07     Meco Man     combine gcc armcc iccarm
 * 2021-02-12     Meco Man     move all definitions located in <clock_time.h> to this file
 */

#ifndef __SYS_TIME_H__
#define __SYS_TIME_H__

#include <rtconfig.h>

#include <sys/types.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* timezone */
/* this method of representing timezones has been abandoned */
#define DST_NONE    0   /* not on dst */

#define RT_LIBC_USING_LIGHT_TZ_DST 1

struct timezone
{
    int tz_minuteswest;   /* minutes west of Greenwich */
    int tz_dsttime;       /* type of dst correction */
};

/* lightweight timezone and daylight saving time */
#ifdef RT_LIBC_USING_LIGHT_TZ_DST
void rt_tz_set(int32_t offset_sec);
int32_t rt_tz_get(void);
int8_t rt_tz_is_dst(void);
#endif /* RT_LIBC_USING_LIGHT_TZ_DST */

int stime(const time_t *t);
time_t timegm(struct tm * const t);
int gettimeofday(struct timeval *tv, struct timezone *tz);
int settimeofday(const struct timeval *tv, const struct timezone *tz);

#if defined(__ARMCC_VERSION) || defined (__ICCARM__) || defined(_WIN32)
struct tm *gmtime_r(const time_t *timep, struct tm *r);
char* asctime_r(const struct tm *t, char *buf);
char *ctime_r(const time_t * tim_p, char * result);
struct tm* localtime_r(const time_t* t, struct tm* r);
#endif /* defined(__ARMCC_VERSION) || defined (__ICCARM__) || defined(_WIN32) */

#ifdef _WIN32
struct tm* gmtime(const time_t* t);
struct tm* localtime(const time_t* t);
time_t mktime(struct tm* const t);
char* ctime(const time_t* tim_p);
time_t time(time_t* t);
#endif /* _WIN32 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _SYS_TIME_H_ */
