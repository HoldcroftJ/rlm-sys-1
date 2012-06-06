/* -*- C -*- ****************************************************************
 *
 *  System        : 
 *  Module        : 
 *  Object Name   : $RCSfile$
 *  Revision      : $Revision$
 *  Date          : $Date$
 *  Author        : $Author$
 *  Created By    : Russ Magee
 *  Created       : Thu May 31 20:49:18 2012
 *  Last Modified : <120605.2218>
 *
 *  Description	
 *
 *  win32 hi-res timer (min. 1ms period)
 *  From an example given here:
 *  http://cygwin.com/ml/cygwin/2007-02/msg00740.html and
 *  http://msdn.microsoft.com/en-us/library/windows/desktop/dd743626%28v=vs.85%29.aspx
 * 
 *  Notes
 *
 *  History
 *	
 ****************************************************************************/
/*
 * To build standalone test executable: [__CYGWIN__]
 * gcc -o win32_timer timer_1ms.c -lwinmm -D__MAINTEST__=1
 */

static const char rcsid[] = "@(#) : $Id$";

#include <stdio.h>

#ifdef __CYGWIN__
#include "windows.h"
#endif

#include "timer_1ms.h"

#ifdef __CYGWIN__

static int32_t
SetSchedulerRes(uint32_t period_ms)
{
    int32_t retVal = 0;
    TIMECAPS tc;
    uint32_t timerRes;
    
    if( timeGetDevCaps(&tc, sizeof(tc)) != MMSYSERR_NOERROR ) {
        printf("timeGetDevCaps error %d\n", GetLastError());
        retVal = -1;
    }
    else {
        printf("tc.wPeriodMin:%u tc.wPeriodMax:%u\n", tc.wPeriodMin), tc.wPeriodMax;
        timerRes = min(max(tc.wPeriodMin, period_ms), tc.wPeriodMax);
        if( timeBeginPeriod(timerRes) != MMSYSERR_NOERROR ) {
            printf("timeBeginPeriod error %d\n", GetLastError());
            retVal = -1;
        }
        else {
            printf("set scheduler res to %ums.\n", timerRes);
        }
    }
    
    return retVal;
}

static int32_t ResetSchedulerRes(uint32_t period_ms) {
    int32_t retVal = 0;
    
    if( timeEndPeriod(period_ms) != MMSYSERR_NOERROR ) {
        printf("error resetting schedule res - error %d\n", GetLastError());
        retVal = -1;
    }
    else {
        printf("reset scheduler res.\n");
    }
    
    return retVal;
}

static int32_t installTimerFunc(uint32_t period_ms, uint32_t *timer_id,
                         void *cb, uint32_t arg) {
    int32_t retVal = 0;
    
    if( (*timer_id = timeSetEvent(period_ms, period_ms,
                     (LPTIMECALLBACK)cb,
                     (DWORD)arg,
                     TIME_PERIODIC)) == 0 ) {
        printf("error installing timer callback - error %d\n", GetLastError());
        retVal = -1;
    }
    else {
        printf("installed timer callback.\n"); fflush(stdout);
    }
    
    return retVal;
}

static int32_t uninstallTimerFunc(uint32_t timer_id) {
    return timeKillEvent(timer_id);
}


#if defined(__MAINTEST__)
static volatile uint32_t counter = 0ul;
static volatile int32_t exitFlag = 0;

/* NOTE it is VITAL that the callback is of type 'void CALLBACK'. Leaving
 * out the CALLBACK typedef results in segfaults. ? -rlm 2012-05-31
 * [CALLBACK is a typedef for __stdcall.]
 */
static void CALLBACK testCallback(uint32_t timerID, uint32_t msg, uint32_t inc, uint32_t resvd1, uint32_t resvd2) {
    counter += (inc ? inc : 1);
    printf("timerID:%u counter:%u\n", timerID, counter); fflush(stdout);
    if( counter >= 100u ) {
        exitFlag = 1;
    }
}

#define TPERIOD 1u /* ms */

int main(int argc, char *argv[]) {
    int32_t status;
    uint32_t timer_id = 0;
    
    status = SetSchedulerRes(TPERIOD);
    status = status ? status : installTimerFunc(TPERIOD, &timer_id, testCallback, 1);
    
    while( exitFlag == 0 ) { usleep(1); }
    
    (void)uninstallTimerFunc(timer_id);
    (void)ResetSchedulerRes(TPERIOD);
    
    return status;
}
#endif /* __MAINTEST__ */

#endif /* __CYGWIN__ */

/** Install ms-resolution timer
 * 
 * @param[out] tid - handle for installed timer
 * @param[in] cb - callback function to execute on each tick
 * @param[in] arg - optional argument to callback
 * @return int32_t - TRUE if timer installed OK; FALSE otherwise
 */
int32_t installMsTimer(uint32_t *tid, void* cb, uint32_t arg)
{
    return installTimerFunc(1, tid, cb, arg);
}

/** Uninstall ms-resolution timer
 *
 * @param[in] tid - timer handle used to install timer
 * @return int32_t - TRUE if timer was uninstalled; FALSE otherwise
 */
int32_t uninstallMsTimer(uint32_t tid)
{
    return uninstallTimerFunc(tid);
}

