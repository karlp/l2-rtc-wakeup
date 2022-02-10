/*
 * Handles the nitty gritty of entering and exiting low power modes,
 * and when that's allowed.
 */
#include <stdio.h>
#include "FreeRTOS.h"

#include <exti/exti.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <pwr/pwr.h>
#include <rcc/rcc.h>
#include <uart/uart.h>

static uint32_t _rcc_cr;
static uint32_t _rcc_cfgr;
auto const _msi_range = 0;

// hackyhack..
extern Pin led_g;

void pre_lpsleep(uint32_t _msi_range)
{
	_rcc_cr = RCC->CR;
	_rcc_cfgr = RCC->CFGR;
#if defined(STM32WB)
	// According to code samples, but not clearly described
	// But not clearly described in the ref man, this call would normally be
	// made by the CPU2 code itself.  Without turning it on and running,
	// we do it ourselves...
	// We can actually set it _lower_ than the cpu1 mode, but this is sufficient...
	// Some single core demos set this up front, once, but we can do it here easily rightnow.
	PWR.set_lpms_c2(4);  // use shutdown all the way?  (no, makes no difference here :(
#endif

//#if defined(STM32WB) // This has no impact on this mode!
//	// Separate section for errata on stop mode and debug
//	EXTI->IMR2 &= ~(1<<(48-32));
//	EXTI->C2IMR2 &= ~(1<<(48-32));
//#endif
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
#if defined(STM32WB)
	// According to code samples, but not clearly described
	// But not clearly described in the ref man, this call would normally be
	// made by the CPU2 code itself.  Without turning it on and running,
	// we do it ourselves...
	// We can actually set it _lower_ than the cpu1 mode, but this is sufficient...
	// Some single core demos set this up front, once, but we can do it here easily rightnow.
	PWR.set_lpms_c2(stop_mode);
#endif

#if 0  // TODO measure this? (but need to then restore properly for lpuart and leds and shit?
	led_g.off();
	// This section is in some demos, we're doing it for now just to try for completeness
	RCC.enable(rcc::GPIOA);
	RCC.enable(rcc::GPIOB);
	RCC.enable(rcc::GPIOC);
	RCC.enable(rcc::GPIOD);
	GPIOA->MODER = 0xffffffff;
	GPIOB->MODER = 0xffffffff;
	GPIOC->MODER = 0xffffffff;
	GPIOD->MODER = 0xffffffff;
	GPIOA->PUPDR = 0;
	GPIOB->PUPDR = 0;
	GPIOC->PUPDR = 0;
	GPIOD->PUPDR = 0;
	RCC.disable(rcc::GPIOA);
	RCC.disable(rcc::GPIOB);
	RCC.disable(rcc::GPIOC);
	RCC.disable(rcc::GPIOD);
#endif

	// This doesn't _seem_ to matter if you unplug the debugger,
	// and doesn't help even if you leave it in, but it's in the errata,
	// so maybe I just got lucky in my testsm, better leave it in.
#if defined(STM32WB) && 1
	// Separate section for errata on stop mode and debug
	EXTI->IMR2 &= ~(1<<(48-32));
	EXTI->C2IMR2 &= ~(1<<(48-32));
#endif
	
	PWR.set_lpms(stop_mode);
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
		while (!(LPUART1->ISR & (1<<6)));
//		pre_lpsleep(0);
		pre_stop(2);
	}

	void myPostSleepFunction(const TickType_t xExpectedIdleTime)
	{
//		post_lpsleep();
		post_stop();
	}
}
