/*
 * Implementations of the freertos required portions for rtc
 * based low power tickless idle, inspired heavily by ST's demo code
 * for the stm32wb, which is heavily inspired by freertos demo code
 */

#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"

#include <os/time.h>  /* laks "os" */

static uint32_t ulTimerCountsForOneTick;

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
			// FIXME kkk LpTimerStart(xExpectedIdleTime);
		}

		/* Enter low power mode */
		// FIXME kkk LpEnter();

		if (xExpectedIdleTime != portMAX_DELAY) {
			/**
			 * Get the number of FreeRTOS ticks that has been suppressed
			 * In the current implementation, this shall be kept in critical section
			 * so that the timer server return the correct elapsed time
			 */
//			FIXME kkk ulCompleteTickPeriods = LpGetElapsedTime();
			ulCompleteTickPeriods = 0; // lol, we didn't really sleeeeep
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