/**
 * EXTI button interrupts on STM32WB nucleo board.
 */
#include <cstdio>

#include <cal/cal.h>
#include <cortex_m/debug.h>
#include <dma/dma.h>
#include <exti/exti.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <pwr/pwr.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
#include <rtc/rtc.h>
#include <syscfg/syscfg.h>
#include <timer/timer.h>
#include <uart/uart.h>

auto led_r3 = GPIOB[1];
auto led_g2 = GPIOB[0];
auto led_b1 = GPIOB[5];
auto sw_1 = GPIOC[4];
auto sw_2 = GPIOD[0];
auto sw_3 = GPIOD[1];

/* disable this with a debugger if you want to measure power sanely
 * You'll then have to watch the ITM channels to see that things do what you think.
 */
bool opt_really_use_leds = true;

/**
 * Run at 32Mhz instead of laks "top speed" by default.
 * 32Mhz is what many ST WB demos run at, and if you don't need more, it's
 * probably a good choice.
 */
void krcc_init32(void) {
	// Prefetch and both caches, plus 1WS for 32MHz
	FLASH->ACR = 0x700 | 1;

	// Enable HSE.
	RCC->CR |= (1<<16);
	while(!(RCC->CR & (1<<17)));
	
	/* No PLL, that just wastes power! */

	//RCC->CFGR &= ~0x3; // do I need this? unlikely at reset? possibly at other times?
	RCC->CFGR |= 0x2;
	while ((RCC->CFGR & (2<<2)) != (2<<2)); // SWS = HSE
	// Leave prescalers alone...
}


static void kexti_init(void) {
	// COnnecting all three switches to all three leds...
	
	// sw_x are on C/D
	RCC.enable(rcc::GPIOC);
	RCC.enable(rcc::GPIOD);

	Pin switches[] = {sw_1, sw_2, sw_3};
	
	// input/pullup at least works...
	for (auto sw : switches) {
		sw.set_mode(Pin::Mode::Input);
		sw.set_pull(Pin::Pull::PullUp);
//		printf("pin portaddr is: %#08x\n", sw.get_portaddr());
		uint32_t lol =  (sw.get_portnum() << ((sw.n & 0x3) * 4));
		printf("pin %u, exticr: %u, port %u, shiftupmul: %u, lolwut: %#lx\n", sw.n, sw.n>>2, sw.get_portnum(), sw.n & 0x3, lol);

		printf("Prior: %#lx\n", SYSCFG->EXTICR[sw.n>>2]);
		SYSCFG->EXTICR[sw.n >> 2] &= ~(0xf << ((sw.n & 0x3) * 4));
		printf("after clearing: %#lx\n", SYSCFG->EXTICR[sw.n>>2]);
		SYSCFG->EXTICR[sw.n >> 2] |= (sw.get_portnum() << ((sw.n & 0x3) * 4));
//		SYSCFG->EXTICR[sw.n >> 2] |= lol;
		printf("after oring in lol: %#lx: %#lx\n", lol, SYSCFG->EXTICR[sw.n>>2]);
		
		EXTI->IMR1 |= (1 << sw.n); // unmask
		EXTI->FTSR1 |= (1 << sw.n); // falling edge
		
	}
	
	// Select sources
	// old manual way..
//	SYSCFG->EXTICR[0] = (3 << 4) | (3<<0);  // PD0 and PD1
//	SYSCFG->EXTICR[1] = 2 << 0;  // PC4

//	EXTI->IMR1 = (1 << sw_1.n) | (1<<sw_2.n) | (1<<sw_3.n);  // unmask interrupt
//	EXTI->FTSR1 = (1 << sw_1.n) | (1 << sw_2.n) | ( 1 << sw_3.n);  // falling
 
	NVIC.enable(interrupt::irq::EXTI0);
	NVIC.enable(interrupt::irq::EXTI1);
	NVIC.enable(interrupt::irq::EXTI4);

}


int main() {
	krcc_init32();
	printf("alive\n");

	// Turn on DWT_CYCNT.  We'll use it ourselves, and pc sampling needs it too.
	DWT->CTRL |= 1;
	
	
	kexti_init();
	printf("exti done...\n");

	if (opt_really_use_leds) {
		RCC.enable(rcc::GPIOB);
		led_b1.set_mode(Pin::Output);
		led_g2.set_mode(Pin::Output);
		led_r3.set_mode(Pin::Output);
	}


	while (1) {
		// This demonstrates that all three gpios are correctly inputs, and all three leds are correctly outputs...
//		led_b1.set(sw_1.get());
//		led_g2.set(sw_2.get());
//		led_r3.set(sw_3.get());
		__asm volatile ("wfi");
	}
	return 0;
}


template <>
void interrupt::handler<interrupt::irq::EXTI0>() {
	EXTI->PR1 |= (1<<sw_2.n);
	ITM->stim_blocking(0, '2');
	led_g2.toggle();
}

template <>
void interrupt::handler<interrupt::irq::EXTI1>() {
	EXTI->PR1 |= (1<<sw_3.n);
	ITM->stim_blocking(0, '3');
	led_r3.toggle();
}

template <>
void interrupt::handler<interrupt::irq::EXTI4>() {
	EXTI->PR1 |= (1<<sw_1.n);
	ITM->stim_blocking(0, '1');
	led_b1.toggle();
}

