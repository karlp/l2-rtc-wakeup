/*
 * Implementations of the freertos required portions for rtc
 * based low power tickless idle, inspired heavily by ST's demo code
 * for the stm32wb, which is heavily inspired by freertos demo code
 */

#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"

#include <interrupt/interrupt.h>
#include <interrupt/nvic.h>
#include <pwr/pwr.h>
#include <os/time.h>  /* laks "os" */
#include <rtc/rtc.h>

static uint32_t ulTimerCountsForOneTick;


static uint32_t save_tts;
static const uint32_t rtc_ticks_us = 1000000/(32768/16);  // FIXME - LSE and LSEDIV is hardcoded here!

/**
 * Naiive, just do a wfi for now.
 * just for easy porting/comparison with timecube.
 */
static void LpEnter(void) {
	// ST does another pointless critical section here, but we know we're already in one...
	
	// ST then does a bit of a switch on what sort of sleep...
	const bool use_sleep = true;
	const bool use_stop = false;
	const bool use_off = false;
	
	if (use_sleep) {
		SCB->SCR &= ~(1<<2);  // clear core deepsleep
		//__WFI();
		__asm volatile ("wfi":::"memory");
		/// And now we wake up...
		return;
	}
	if (use_stop) {
		// FIXME - lots of work on making sure CPU2 stops properly too
		// essential eventually, but not now!
		configASSERT(use_stop==false);
		return;
	}
	if (use_off) {
		// FIXME - off mode is actually easier, as you're basically resetting.
		configASSERT(use_off == false);
		return;
	}	
}


/**
 * How long did we actually sleep for? (may have been less than requested!)
 * FIXME kkk LOL, we're not handling rounding errors in converting between rtc clocks and rtos ticks here!
 * See how ST handles it properly, accumlating and correcting over time on this return path..
 * @return 
 */
static uint32_t LpGetElapsedTime(void) {
	// FIXME lol
	//configASSERT(false);
	// WUTF, assume we slept for how long we asked for...
	if (RTC->ISR & (1<<10)) {
		return save_tts;
	} else {
		// FXME - this isn't even correct, it's a down coutner, we need to save our loaded value too!
//		auto slept_us = RTC->WUTR * rtc_ticks_us;
//		// so, 500ms requested, results in 1024 rtc ticks.
//		// say we woke up with 800 rtc ticks left...
//		// 800 * our constant gives us 
//		// now we want it in ticks, which is like, 1/1000, 
//		// so, if we asked for 
//		return slept_us / configTICK_RATE_HZ / 1000;
		//configASSERT(false);  // lol fixme
		printf("lol failed to wutf!\n");
	}
	return save_tts;
}

/**
 * try and configure an RTC wakeup after x ticks.
 * @param time_to_sleep in OS ticks.
 */
static void LpTimerStart(uint32_t time_to_sleep) {
	save_tts = time_to_sleep;

//  uint64_t time;
//
//  /* Converts the number of FreeRTOS ticks into hw timer tick */
//
//  time = (time_to_sleep * 1000 * 1000 );
//  time = time / ( CFG_TS_TICK_VAL * configTICK_RATE_HZ );
	// THis is the RTC tick rate in usecs, * OS ticks in seconds, so you get a 
	
	// so for 1000Hz OS "tick" (yes, tickless just means we don't have IRQs for un-needed ticks)
	// we have a 1000us per tick.
	// RTC is clocking at 32768/16 => 
	// 30.517xxx usecs * 16 => 488.28125 usecs...
	// later, we see how ST fixes up that remainder!
	// so, for a requested OS sleep of 100 ticks (==> 100ms ==> 100 000 usecs)
	// we want to sleep for howmany rtc ticks?
	// 100 000 / 488.28125.  (ST does it in integer though, 
	// ST makes this complicated and uses rounding up/down/floor/ceiling, and I think it's unnecessary
	// unless you are concerned with the tick val becoming between 0 and 1, and you want to avoid "0"
	
	// I just want to have ticks in "rtos ticks" and convert into "rtc ticks"
	// ST does some * 1e6 which I suspect is to avoid numerical truncation in lots of places, keeping things as integers...
	
	// I'd rather just.... assume that rtc ticks are "smaller" and just make it a multiplication all the time?
	// FIXME - we need to account for the rounding error here later!  ST does that properly, without it, we'll drift quite badly...
	auto desired_us = time_to_sleep * 1000000 / configTICK_RATE_HZ;
	
	auto rtc_ticks_needed = desired_us / rtc_ticks_us;
	
	
	// FIXME kkk HW_TS_Start(rtc_ricks_needed);
	
	// Ok... rtc wakeup now please...
//	RTC.unlock();
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	RTC->CR &= ~(1<<10); // clear WUTE while we setup
	while (!(RTC->ISR & (1<<2))) {
		; // Wait for WUTWF
	}
	printf("sleep rtc ticks: %lu\n", rtc_ticks_needed);
	RTC->WUTR = rtc_ticks_needed;
	RTC->CR |= (1<<14) | (1<<10); // WUTIE | WUTE
	
	PWR->CR3 |= (1<<15); // Enable internal wakeup line.
//	RTC.lock();
	RTC->WPR=0xff;
	
//
//  HW_TS_Start(LpTimerContext.LpTimerFreeRTOS_Id, (uint32_t)time);
	
//
//  /**
//   * There might be other timers already running in the timer server that may elapse
//   * before this one.
//   * Store how long before the next event so that on wakeup, it will be possible to calculate
//   * how long the tick has been suppressed
//   */
//  LpTimerContext.LpTimeLeftOnEntry = HW_TS_RTC_ReadLeftTicksToCount( );
//
//  return;
	
}

static void lPortSetupTimerInterrupt(void)
{
	printf("lol, setup!");
	/* Calculate the constants required to configure the tick interrupt. */
	ulTimerCountsForOneTick = (configCPU_CLOCK_HZ / configTICK_RATE_HZ);

	/* Stop and clear the SysTick. */
	//	portNVIC_SYSTICK_CTRL_REG = 0UL;
	//	portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
	STK.CTRL = 0;
	STK.VAL = 0;

	/* Configure SysTick to interrupt at the requested rate. */
	//	portNVIC_SYSTICK_LOAD_REG = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
	//	portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT );	
	STK.LOAD = (configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1;
	// Clock at sysclock, interrupts, and enable!
	STK.CTRL = (1 << 2) | (1 << 1) | (1 << 0);
}

static void lPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
	printf("lol, sleepingfor: %lu\n", xExpectedIdleTime);
	/* If low power is not used, do not stop the SysTick and continue execution */
	/**
	 * Although this is not documented as such, when xExpectedIdleTime = 0xFFFFFFFF = (~0),
	 * it likely means the system may enter low power for ever ( from a FreeRTOS point of view ).
	 * Otherwise, for a FreeRTOS tick set to 1ms, that would mean it is requested to wakeup in 8 years from now.
	 * When the system may enter low power mode for ever, FreeRTOS is not really interested to maintain a
	 * systick count and when the system exits from low power mode, there is no need to update the count with
	 * the time spent in low power mode
	 */
	uint32_t ulCompleteTickPeriods;

	/* Stop the SysTick  to avoid the interrupt to occur while in the critical section.
	 * Otherwise, this will prevent the device to enter low power mode
	 * At this time, an update of the systick will not be considered
	 *
	 */
	STK.CTRL &= ~(1 << 0); // Disable;

	/* Enter a critical section but don't use the taskENTER_CRITICAL()
	 * method as that will mask interrupts that should exit sleep mode. */
	__asm volatile ( "cpsid i" :: : "memory");
	__asm volatile ( "dsb");
	__asm volatile ( "isb");

	/* If a context switch is pending or a task is waiting for the scheduler
	      to be unsuspended then abandon the low power entry. */
	if (eTaskConfirmSleepModeStatus() == eAbortSleep) {
		/* Restart SysTick. */
//		portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
		STK.CTRL |= (1<<0);

		/* Re-enable interrupts - see comments above __disable_interrupt()
			    call above. */
		__asm volatile ( "cpsie i" ::: "memory" );
	} else {
		if (xExpectedIdleTime != portMAX_DELAY) {
			/* Remove one tick to wake up before the event occurs */
			xExpectedIdleTime--;
			/* Start the low power timer */
			LpTimerStart(xExpectedIdleTime);
		}

		/* Enter low power mode */
		LpEnter();

		if (xExpectedIdleTime != portMAX_DELAY) {
			/**
			 * Get the number of FreeRTOS ticks that has been suppressed
			 * In the current implementation, this shall be kept in critical section
			 * so that the timer server return the correct elapsed time
			 */
			ulCompleteTickPeriods = LpGetElapsedTime();
//			ulCompleteTickPeriods = 0; // lol, we didn't really sleeeeep
			vTaskStepTick(ulCompleteTickPeriods);
		}

		/* Restart SysTick */
//		portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
//		portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
//		portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;
		STK.VAL = 0;
		STK.CTRL |= (1<<0);
		STK.LOAD = ulTimerCountsForOneTick - 1;

		/* Exit with interrupts enabled. */
		__asm volatile ( "cpsie i" ::: "memory" );
	}

}

extern "C" {

	void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
	{
		lPortSuppressTicksAndSleep(xExpectedIdleTime);
	}

	void vPortSetupTimerInterrupt(void)
	{
		lPortSetupTimerInterrupt();
	}
}