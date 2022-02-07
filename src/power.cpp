/*
 * Handles the nitty gritty of entering and exiting low power modes,
 * and when that's allowed.
 */
#include <stdio.h>
#include "FreeRTOS.h"

#include <interrupt/interrupt.h>
#include <rcc/rcc.h>
#include <pwr/pwr.h>

static uint32_t _rcc_cr;
static uint32_t _rcc_cfgr;
auto const _msi_range = 0;

void pre_lpsleep(uint32_t _msi_range)
{
	_rcc_cr = RCC->CR;
	_rcc_cfgr = RCC->CFGR;
	// Ensure MSI is on, and switch down to that range...
	RCC->CR |= (1 << 3) | 1; // Also enable range selection here
	// Can't modify MSI range when it's on and not ready
	while (!(RCC->CR & (1 << 1)));
	RCC->CR &= ~(0xf << 4);
	RCC->CR |= (_msi_range << 4);

	RCC->CFGR &= ~(0x3 << 0); // Set MSI as clock source
	// SLEEPDEEP must be clear
	SCB->SCR &= ~(1 << 2);

	PWR->CR1 |= (1 << 14); // Set regulator low power mode
}

void post_lpsleep(void)
{
	// Restore regulator power
	PWR->CR1 &= ~(1 << 14);
	while (PWR->SR2 & (1 << 9)); // Wait for REGLPF

	// restore system clock settings
	RCC->CR = _rcc_cr;
	RCC->CFGR = _rcc_cfgr;
}

void pre_stop(uint32_t stop_mode)
{
	//		printf("stop pre %lu\n", _mode);
	_rcc_cr = RCC->CR;
	_rcc_cfgr = RCC->CFGR;
	PWR.set_lpms(stop_mode);
	//		PWR.set_lpms_c2(_mode);
	//		if ((PWR->EXTSCR >> 8) & 0xf) {
	//			// if any of thos fucking flags is set, clear them all!
	//			PWR->EXTSCR |= 0x3;
	//		}
	// Enable SLEEPDEEP
	SCB->SCR |= (1 << 2);
}

void post_stop(void)
{
	//		printf("stop post\n");
	// 3. We wakeup with either HSI16 or MSI depending on RCC->CFGR:STOPWUCK...
	// we're just going to to with "restore RCC->CR and RCC-CFGR....
	RCC->CR = _rcc_cr;
	RCC->CFGR = _rcc_cfgr;
}

extern "C" {

	void myPreSleepFunction(TickType_t xModifiableIdleTime)
	{
		pre_lpsleep(0);
//		pre_stop(2);
	}

	void myPostSleepFunction(const TickType_t xExpectedIdleTime)
	{
		post_lpsleep();
//		post_stop();
	}
}
