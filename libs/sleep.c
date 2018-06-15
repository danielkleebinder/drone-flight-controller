/*
 * sleep.c
 *
 *  Created on: 12.06.2018
 *      Author: Daniel Kleebinder
 */


#include "sleep.h"

#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <inc/hw_memmap.h>



uint8_t initializedSleepTimer = false;


/**
 * Initializes and enables the sleep function.
 */
void initializeSleep() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_PERIODIC);
    TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_PIOSC);
    initializedSleepTimer = true;
}


/**
 * Sleeps the given amount of time.
 *
 * @param ms Time in milliseconds.
 */
void sleep(uint16_t ms) {
    // Initialize sleep timer
    if (!initializedSleepTimer) {
        initializeSleep();
    }

    // Fork and join sleep
    if (ms >= 655) {
        sleep(ms / 2);
        sleep(ms / 2);
        return;
    }

    // Start sleeping process
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 150);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 100 * ms);

    TimerEnable(TIMER0_BASE, TIMER_A);

    uint32_t status = 0;
    do {
        status = TimerIntStatus(TIMER0_BASE, false);
    } while(!(status & TIMER_TIMA_TIMEOUT));
    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerIntClear(TIMER0_BASE, status);
}
