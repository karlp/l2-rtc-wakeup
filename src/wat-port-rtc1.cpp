/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <cstdio>

#include <exti/exti.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <pwr/pwr.h>
#include <rcc/rcc.h>
#include <rtc/rtc.h>
#include <syscfg/syscfg.h>

#if defined(STM32WB)
auto led_r = GPIOB[1];
auto led_g = GPIOB[0];
auto led_b = GPIOB[5];
auto led = led_g;
auto sw_1 = GPIOC[4];
auto sw_2 = GPIOD[0];
auto sw_3 = GPIOD[1];
#elif defined(STM32L4)
auto led_b = GPIOA[5];
auto sw_3 = GPIOC[13];
auto led = led_b;
#else
#error "Unhandled platform"
#endif


static void kgpio_init(void) {


	RCC.enable(rcc::GPIOB);
	RCC.enable(rcc::GPIOD);

	sw_3.set_mode(Pin::Mode::Input);
	sw_3.set_pull(Pin::Pull::PullUp);
	led.set_mode(Pin::Output);


	SYSCFG->EXTICR[sw_3.n >> 2] &= ~(0xf << ((sw_3.n & 0x3) * 4));
	SYSCFG->EXTICR[sw_3.n >> 2] |= (sw_3.get_portnum() << ((sw_3.n & 0x3) * 4));

	EXTI->IMR1 |= (1 << sw_3.n); // unmask
	EXTI->FTSR1 |= (1 << sw_3.n); // falling edge

#if defined(STM32WB)
	NVIC.enable(interrupt::irq::EXTI1);
#elif defined(STM32L4)
	NVIC.enable(interrupt::irq::EXTI15_10);
#endif

}

static void krtc_init(void)
{
	// Use the last backup register as a flag that we're already initialized
	if (RTC->BKP[19] == 0xcafe) {
		printf("RTC: already setup!\n");
		return;
	}
	PWR->CR1 |= (1 << 8); // Unlock backup domain

	// We've decided that it's "not setup" so go ahead and hard reset
	// The RTC.  You want to be careful with this, or you lose your
	// precious calendar.
	printf("hard reset backup domain\n");
	RCC->BDCR |= (1 << 16);
	RCC->BDCR &= ~(1 << 16);

	// make sure LSE is on, we need that for most sane uses of RTC
	RCC->BDCR |= 1;
	while (!(RCC->BDCR & (1 << 1))) {
		; // wait for LSERDY
	}

	RCC->BDCR |= (1 << 8); // RTCSEL == LSE
	RCC->BDCR |= (1 << 15); // RTCEN
	RCC.enable(rcc::RTCAPB);
	// Probably still an issue on L4, but it hangs here...
	while (!(RCC->APB1ENR1 & rcc::RTCAPB)) {
		; // make sure we have access!
	}

	RTC.unlock();

	RTC->PRER = 0x7f00ff; // default, gives 1sec from 32768

	// setup calendar as well
	RTC->ISR = (1 << 7);
	while (!(RTC->ISR & (1 << 6))) {
		; // Wait for INITF
	}
	RTC->CR &= ~(1 << 6); // 24hr format.
	RTC->DR = 0x220127 | (4 << 13); // Thursday,2022-01-27.
	RTC->TR = 0x092042; // 09:20:42 AM
	RTC->ISR &= ~(1 << 7); // Clear init, start calendar

	// set wakekup clock sel to rtc/16, so ~488usecs per tick
	RTC->CR &= ~(0x7 << 0);
	RTC->CR |= (0 << 0);

	RTC.lock();

	// Set it up now, but we will turn it on later.
	// RTC Wakeup is via exti 19! only listed in the nvic section
	EXTI->RTSR1 |= (1 << 19);
	EXTI->IMR1 |= (1 << 19);
	// we want to wakeup, but we don't need a handler
	//NVIC.enable(interrupt::irq::RTC_WKUP);
	RTC->BKP[19] = 0xcafe;
}

static void krcc_init(void) {
	RCC->CR |= (1<<8); // HSI on

	// cpu1 sysclock / 1
	RCC->CFGR &= ~(0xf<<4);
	RCC->CFGR |= (0<<4);

	// cpu2 sysclock / 2
	RCC->EXTCFGR &= ~(0xf<<4);
	RCC->EXTCFGR |= (0x8<<4);

	// SYSClock to HSI
	RCC->CFGR &= ~(0x3);
	RCC->CFGR |= (1<<0);

	// AHB4 (shared) to /1
	RCC->EXTCFGR &= ~(0xf<<0);

	// APB1 to div1
	// APB2 to div1
	RCC->CFGR &= ~((0x7<<11) | (0x7<<8));
	// ignore the LL tick shit
}

static void print_date(void)
{
	// order of read is important!
	uint32_t s = RTC->SSR;
	uint32_t t = RTC->TR;
	uint32_t d = RTC->DR;

	printf("20%02lx:%02lx:%02lx T %02lx:%02lx:%02lx.%03ld\n",
		(d >> 16) & 0xff,
		(d >> 8) & 0x1f,
		d & 0x3f,
		(t >> 16) & 0x3f,
		(t >> 8) & 0x7f,
		t & 0x7f,
		s * 1000 / 256 // based on "default" fck_spre
		);
}

static volatile bool pressed = false;


int main() {
	krcc_init();
	kgpio_init();
	krtc_init();

	if (PWR->EXTSCR & (1<<8) && (PWR->EXTSCR & (1<<10))) {
		print_date();
		printf("was standby\n");

//		LL_PWR_ClearFlag_C1STOP_C1STB();
//		LL_PWR_ClearFlag_C2STOP_C2STB();
		PWR->EXTSCR |= (3<<0);

		RTC->ISR = ~(1<<10); // Clears rtc wakeup, only if still locked

		while(1) {
			for (auto i = 0; i < 0x40000; i++) {
				asm volatile ("nop");
			}
			led.toggle();
		}
	} else {
		printf("blah\n");
		while (!pressed) {
			for (auto i = 0; i < 0x40000; i++) {
				asm volatile ("nop");
			}
			led.toggle();
		}

		printf("going to sleep!\n");

		// ensure EIWUL is set
		PWR->CR3 |= (1<<15);
		PWR.set_lpms(3); // standby
		PWR.set_lpms_c2(3); // Standby

		SCB->SCR |= (1<<2); // deepsleep

		asm volatile ("wfi");
	}
	
	

}

#if defined(STM32WB)
template <>
void interrupt::handler<interrupt::irq::EXTI1>()
#elif defined(STM32L4)
template <>
void interrupt::handler<interrupt::irq::EXTI15_10>()
#else
#error "unsupported platform"
#endif
{
	EXTI->PR1 |= (1 << sw_3.n);
	pressed = true;
}

