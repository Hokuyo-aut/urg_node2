/*!
  \file
  \~japanese
  \brief タイムスタンプの取得
  \~english
  \brief Gets computer timestamp
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#include "ticks.h"
#include "detect_os.h"
#include <time.h>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

void gettime(struct timespec *ts)
{
  #ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  ts->tv_sec = mts.tv_sec;
  ts->tv_nsec = mts.tv_nsec;

  #else
  #ifndef QRK_WINDOWS_OS
  clock_gettime(CLOCK_REALTIME, ts);
  #endif
  #endif
}


long qrk::ticks(void)
{
    static bool is_initialized = false;
#if defined(QRK_WINDOWS_OS)
    clock_t current_clock;
#else
    static struct timespec first_spec;
    struct timespec current_spec;
#endif
    long msec_time;

#if defined(QRK_WINDOWS_OS)
    if (!is_initialized) {
        is_initialized = true;
    }
    current_clock = clock();
    msec_time = current_clock / (CLOCKS_PER_SEC / 1000);
#else
    if (!is_initialized) {
        gettime(&first_spec);
        is_initialized = true;
    }
    gettime(&current_spec);
    msec_time =
        (current_spec.tv_sec - first_spec.tv_sec) * 1000
        + (current_spec.tv_nsec - first_spec.tv_nsec) / 1000000;
#endif
    return msec_time;
}
