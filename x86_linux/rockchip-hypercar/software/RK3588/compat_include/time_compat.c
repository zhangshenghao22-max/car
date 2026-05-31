#include <rtthread.h>
#include <sys/time.h>
#include <time.h>

#ifndef NANOSECOND_PER_SECOND
#define NANOSECOND_PER_SECOND 1000000000UL
#endif

#ifndef MICROSECOND_PER_SECOND
#define MICROSECOND_PER_SECOND 1000000UL
#endif

static const short days_before_month[13] =
{
    0,
    31,
    31 + 28,
    31 + 28 + 31,
    31 + 28 + 31 + 30,
    31 + 28 + 31 + 30 + 31,
    31 + 28 + 31 + 30 + 31 + 30,
    31 + 28 + 31 + 30 + 31 + 30 + 31,
    31 + 28 + 31 + 30 + 31 + 30 + 31 + 31,
    31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30,
    31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31,
    31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30,
    31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30 + 31,
};

static int is_leap_year(int year)
{
    return (!(year % 4) && ((year % 100) || !(year % 400)));
}

int32_t rt_tz_get(void)
{
    return 0;
}

time_t timegm(struct tm * const t)
{
    time_t years;
    time_t days;
    time_t century_years;

    if (!t || t->tm_year < 70 || t->tm_mon < 0 || t->tm_mon > 11)
    {
        return (time_t)-1;
    }

    years = (time_t)t->tm_year - 70;
    days = years * 365 + (years + 1) / 4;

    if (years >= 131)
    {
        century_years = (years - 131) / 100;
        days -= (century_years >> 2) * 3 + 1;
        century_years &= 3;
        if (century_years == 3)
        {
            century_years--;
        }
        days -= century_years;
    }

    days += days_before_month[t->tm_mon] + t->tm_mday - 1;
    if (t->tm_mon > 1 && is_leap_year(t->tm_year + 1900))
    {
        days++;
    }

    return (((days * 24) + t->tm_hour) * 60 + t->tm_min) * 60 + t->tm_sec;
}

int clock_gettime(clockid_t clock_id, struct timespec *tp)
{
    rt_uint64_t ns;

    if (!tp)
    {
        return -1;
    }

    RT_UNUSED(clock_id);

    ns = (rt_uint64_t)rt_tick_get_millisecond() * 1000000ULL;
    tp->tv_sec = (time_t)(ns / NANOSECOND_PER_SECOND);
    tp->tv_nsec = (long)(ns % NANOSECOND_PER_SECOND);

    return 0;
}

int gettimeofday(struct timeval *tv, void *tz)
{
    rt_uint64_t us;

    RT_UNUSED(tz);

    if (!tv)
    {
        return -1;
    }

    us = (rt_uint64_t)rt_tick_get_millisecond() * 1000ULL;
    tv->tv_sec = (time_t)(us / MICROSECOND_PER_SECOND);
    tv->tv_usec = (suseconds_t)(us % MICROSECOND_PER_SECOND);

    return 0;
}
