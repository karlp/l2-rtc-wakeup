/**
 * "example" of using the RTC wakeup timer.
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

auto led_r = GPIOB[1];
auto led_g = GPIOB[0];
auto led_b = GPIOB[5];
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

/** We're going to use this for low power wakeups! see l_freertos.cpp....  (code organization wat?
 * Big difference to ST implementation, we're _ONLY_ going to use it for low power sleep wakeups
 * Because we're _only_ supporting freertos, we don't need to have linked lists
 * of overlapping timers all using this rtc wakeup timer.  just use the os timers for that,
 * and it actually turns into the same thing, handled internally.
 * this makes it _wayyyyyy simpler_ but also much less "flexible"
 */
static void krtc_init(void) {
	PWR->CR1 |= (1<<8); // Unlock backup domain
	
	//FUcking wat? (this is st style)
//	if (!(RCC->BDCR & (1<<1))) { // LSERDY
		// Force a backup domain reset...
		RCC->BDCR |= (1<<16);
		RCC->BDCR &= ~(1<<16);
		RCC->BDCR |= (1<<0); // LSE on...

		while (!(RCC->BDCR & (1<<1))) {
			; // LSE RDY
		}

		
		RCC->BDCR |= (1<<8); // RTCSEL = LSE
		
//	}
	
	
////	RCC->BDCR = (1<<15) | (1<<8) | (1<<0); // RTCEN | RTCSEL=LSE | LSE ON
//	// no difference splitting like ST does...
//	RCC->BDCR |= (1<<8); // RTCSEL = LSE
//	RCC->BDCR |= (1<<15); // RTCEN
//	RCC.enable(rcc::RTCAPB);  // APB access, apparently important too...
//
////	RTC->CR |= (1<<5); // Bypass shadow ? why do we care?
	
	// Now, MX_RTC_Init()
	RCC->BDCR |= (1<<15); // RTCEN
	RCC.enable(rcc::RTCAPB);  // ST has an error in this, but, whatever, obviously not very important
	
	// Ok, now rtc itself..
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
//	RTC.unlock();
	
	
	//// Might as well init calendar? 
	//RTC->ISR |= (1<<7); // set init
	RTC->ISR = 0xffffffff; // ST style?!
	while (!(RTC->ISR & (1<<6))) {   // FIXME - this always hangs....
		; // Poll til ready.
	}
	RTC->PRER = 0x7f00ff;  // yes, default is divided by (0x7f+1)*(0xff+1) => 32768....
	
	// zero time is as good as any....
	RTC->TR = 0;
	RTC->DR = 0;
	
	RTC->CR &= ~(1<<6);  // 24 hour yo
	RTC->ISR &= ~(1<<7);  // Clear init to start calendar
	///// End of "calendar"
	
	//// WAKEUP TIMER _init_ but don't start...
	// WUCKSEL = 16 is good...
	RTC->CR &= ~(1<<10); // clear WUTE while we setup
	while (!(RTC->ISR & (1<<2))) {
		; // Wait for WUTWF
	}
	RTC->CR &= ~(0x7<<0);  // Clear WUCKSEL
	RTC->CR |= (0<<0); // WUCKSEL = RTC/16...
	// I _think_ we can start it here, but jsut don't turn on the interrupt?
	RTC->CR |= (1<<10); // WUTE
	// But WUTR is protected by WUTWF anyway, so probably doesn't buy us much...
	
	
//	RTC.lock();
	RTC->WPR=0xff;
	
	
	// Oh yeah, someone has to allow the RTC wakeup timer irq in nvic to make it wakeu right?
	NVIC.enable(interrupt::irq::RTC_WKUP);
}


static void kexti_init(void) {
	// sw_3.. is D1...
	RCC.enable(rcc::GPIOD);

	sw_3.set_mode(Pin::Mode::Input);
	sw_3.set_pull(Pin::Pull::PullUp);
	
	SYSCFG->EXTICR[sw_3.n >> 2] &= ~(0xf << ((sw_3.n & 0x3) * 4));
	SYSCFG->EXTICR[sw_3.n >> 2] |= (sw_3.get_portnum() << ((sw_3.n & 0x3) * 4));

	EXTI->IMR1 |= (1 << sw_3.n); // unmask
	EXTI->FTSR1 |= (1 << sw_3.n); // falling edge
	
	NVIC.enable(interrupt::irq::EXTI1);
}

static bool pressed = true;

int main() {
	krcc_init32();
	printf("clocked up, turning on rtc...\n");

	// Turn on DWT_CYCNT.  We'll use it ourselves, and pc sampling needs it too.
	DWT->CTRL |= 1;
	
//	krtc_init();
//	printf("RTC init, ready to start!\n");
	
	kexti_init();
	

	if (opt_really_use_leds) {
		RCC.enable(rcc::GPIOB);
		led_b.set_mode(Pin::Output);
	}


	while (1) {
		int mul = pressed ? 1 : 3;
		for (int i = 0; i < 0x80000 * mul; i++) {
			asm volatile ("nop");
		}
		if (opt_really_use_leds) {
//			led_b.set(sw_3.get());
			led_b.toggle();
		}
	}
	return 0;
}


template <>
void interrupt::handler<interrupt::irq::EXTI1>() {
	EXTI->PR1 |= (1<<sw_3.n);
	ITM->stim_blocking(0, '!');
	pressed = !pressed;
	//opt_really_use_leds = !opt_really_use_leds;
}

