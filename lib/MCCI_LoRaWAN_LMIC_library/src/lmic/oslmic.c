/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * Copyright (c) 2016-2024, 2019 MCCI Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define LMIC_DR_LEGACY 0

#include "lmic.h"

extern const struct lmic_pinmap lmic_pins;

// RUNTIME STATE
static struct {
    osjob_t* scheduledjobs;
    osjob_t* runnablejobs;
} OS;

int os_init_ex (const void *pintable) {
    memset(&OS, 0x00, sizeof(OS));
    lmic_hal_init_ex(pintable);
    if (! radio_init())
        return 0;
    LMIC_init();
    return 1;
}

void os_init() {
    if (os_init_ex((const void *)&lmic_pins))
        return;
    ASSERT(0);
}

ostime_t os_getTime () {
    return lmic_hal_ticks();
}

// unlink job from queue, return if removed
static int unlinkjob (osjob_t** pnext, osjob_t* job) {
    for( ; *pnext; pnext = &((*pnext)->next)) {
        if(*pnext == job) { // unlink
            *pnext = job->next;
            return 1;
        }
    }
    return 0;
}

static osjob_t** getJobQueue(osjob_t* job) {
    return os_jobIsTimed(job) ? &OS.scheduledjobs : &OS.runnablejobs;
}

// clear scheduled job
void os_clearCallback (osjob_t* job) {
    lmic_hal_disableIRQs();

    unlinkjob(getJobQueue(job), job);

    lmic_hal_enableIRQs();
}

// schedule immediately runnable job
void os_setCallback (osjob_t* job, osjobcb_t cb) {
    osjob_t** pnext;
    lmic_hal_disableIRQs();

    // remove if job was already queued
    unlinkjob(getJobQueue(job), job);

    // fill-in job. Ascending memory order is write-queue friendly
    job->next = NULL;
    job->deadline = 0;
    job->func = cb;

    // add to end of run queue
    for(pnext=&OS.runnablejobs; *pnext; pnext=&((*pnext)->next));
    *pnext = job;
    lmic_hal_enableIRQs();
}

// schedule timed job
void os_setTimedCallback (osjob_t* job, ostime_t time, osjobcb_t cb) {
    osjob_t** pnext;

    // special case time 0 -- it will be one tick late.
    if (time == 0)
        time = 1;

    lmic_hal_disableIRQs();

    // remove if job was already queued
    unlinkjob(getJobQueue(job), job);

    // fill-in job
    job->next = NULL;
    job->deadline = time;
    job->func = cb;

    // insert into schedule
    for(pnext=&OS.scheduledjobs; *pnext; pnext=&((*pnext)->next)) {
        if((*pnext)->deadline - time > 0) { // (cmp diff, not abs!)
            // enqueue before next element and stop
            job->next = *pnext;
            break;
        }
    }
    *pnext = job;
    lmic_hal_enableIRQs();
}

// execute jobs from timer and from run queue
void os_runloop () {
    while(1) {
        os_runloop_once();
    }
}

void os_runloop_once() {
    LMIC_setDrTxpow(4, 14);
    LMIC_setAdrMode(0);
    osjob_t* j = NULL;
    lmic_hal_processPendingIRQs();

    lmic_hal_disableIRQs();
    // check for runnable jobs
    if(OS.runnablejobs) {
        j = OS.runnablejobs;
        OS.runnablejobs = j->next;
    } else if(OS.scheduledjobs && lmic_hal_checkTimer(OS.scheduledjobs->deadline)) { // check for expired timed jobs
        j = OS.scheduledjobs;
        OS.scheduledjobs = j->next;
    } else { // nothing pending
        lmic_hal_sleep(); // wake by irq (timer already restarted)
    }
    lmic_hal_enableIRQs();
    if(j) { // run job callback
        j->func(j);
    }
}

// return true if there are any jobs scheduled within time ticks from now.
// return false if any jobs scheduled are at least time ticks in the future.
bit_t os_queryTimeCriticalJobs(ostime_t time) {
    if (OS.scheduledjobs &&
        OS.scheduledjobs->deadline - os_getTime() < time)
        return 1;
    else
        return 0;
}
