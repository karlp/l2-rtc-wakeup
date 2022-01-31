/**
 * "example" of using the RTC wakeup timer.
 * This is like the "RTC_ExitStandbyWithWakeupTimer" example.
 * Blinks on boot, then button SW3 to enter a low power mode, wakesup again later.
 * Extension is that the button can _also_ wake you up early,
 * and you can see how long we actually slept using the RTC itself.
 */
#include <cstdio>

#include <cal/cal.h>
#include <cortex_m/debug.h>
#include <exti/exti.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <pwr/pwr.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
#include <rtc/rtc.h>
#include <syscfg/syscfg.h>

auto led_r = GPIOB[1];
auto led_g = GPIOB[0];
auto led_b = GPIOB[5];
auto sw_1 = GPIOC[4];
auto sw_2 = GPIOD[0];
auto sw_3 = GPIOD[1];
auto led = led_g;

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


static void krtc_init(void) {
	PWR->CR1 |= (1<<8); // Unlock backup domain
	
	// For _now_ we're doing a hard backup domain reset, so we can be sure shit works.
	// in real world, you almost definitely don't want to do that, as you'll trash your RTC that carefully
	// stayed running while you were stopped. ST uses the "Is LSE on?"
	// heuristic, but you could also use a backup register as a flag.
	bool opt_hard_reset_backup_domain = true;
	if (opt_hard_reset_backup_domain) {
		printf("hard reset backup domain\n");
		RCC->BDCR |= (1<<16);
		RCC->BDCR &= ~(1<<16);
	}
	
	// make sure LSE is on, we need that for most sane uses of RTC
	RCC->BDCR |= 1;
	while (!(RCC->BDCR & (1<<1))) {
		; // wait for LSERDY
	}
	
	RCC->BDCR |= (1<<8); // RTCSEL == LSE
	RCC->BDCR |= (1<<15); // RTCEN
	RCC.enable(rcc::RTCAPB);
	while (!(RCC->APB1ENR1 & rcc::RTCAPB)) {
		; // make sure we have access!
	}
	
	RTC.unlock();
	RTC->PRER = 0x7f00ff; // default, gives 1sec from 32768
	
	// set wakekup clock sel to rtc/16, so ~488usecs per tick
	RTC->CR &= ~(0x7<<0);
	RTC->CR |= (0<<0);

	RTC.lock();
	
	// Set it up now, but we will turn it on later.
	// RTC Wakeup is via exti 19! only listed in the nvic section
	EXTI->RTSR1 |= (1<<19);
	EXTI->IMR1 |= (1 << 19);
	NVIC.enable(interrupt::irq::RTC_WKUP);

}

static void kwkup_stop(void) {
	RTC.unlock();
	RTC->CR &= ~(1<<10); // Disable WUTE
	RTC.lock();
}

static void kwkup_start(uint16_t wu_ticks)
{
	RTC.unlock();
	
	// Disable wakeup
	RTC->CR &= ~(1<<10); // Disable WUTE
	while (!(RTC->ISR & (1<<2))) {
		; // wait for WUTWF
	}
	
	RTC->WUTR = wu_ticks;  // ~about 5 seconds.

//	RTC->CR |= (1<<14); // WUTIE enable irq
//	RTC->CR |= (1<<10); // Re-enable WUTE
	RTC->CR |= (1<<14) | (1<<10);  // Re-enable WUTE + WUTIE
	
	RTC.lock();
	
	// now, clear the WUTF flag!
//	RTC->ISR = ~((1<<10) | (1<<7)); // write 0 to WUTF, and ensure INIT stays 0 too...
	RTC->ISR = ~(1<<10); // Clear WUTF. (other bits are protected)

	PWR->CR3 |= (1<<15); // EIWUL enable wakeup cpu1
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

// for starters...
static void ksleep() {
	kwkup_start(9999);
	
	asm volatile ("wfi");
}

static void print_date(void) {
	uint32_t s = RTC->SSR;
	uint32_t t = RTC->TR;
	uint32_t d = RTC->DR;
	printf("20%02lx:%02lx:%02lx T %02lx:%02lx:%02lx.%05ld\n",
		(d >> 16) & 0xff,
		(d >> 8) & 0x1f,
		d & 0x3f,
		(t >> 16) & 0x3f,
		(t >> 8) & 0x7f,
		t & 0x7f,
		s
		);
}

static volatile bool pressed = false;

int main() {
	krcc_init32();
	printf("Started from scratch, clocked up\n");

	// Turn on DWT_CYCNT.  We'll use it ourselves, and pc sampling needs it too.
	DWT->CTRL |= 1;
	
	krtc_init();
	printf("RTC init, ready to  start!\n");
	
	kexti_init();
	

	if (opt_really_use_leds) {
		RCC.enable(rcc::GPIOB);
		led.set_mode(Pin::Output);
	}

	// Depending on sleep mode, you may be restarting from scratch, or looping
	while (1) {
		printf("start loop\n");
		while (!pressed) {
			for (int i = 0; i < 0x80000; i++) {
				asm volatile ("nop");
			}
			if (opt_really_use_leds) {
				led.toggle();
			}
		}
		// Ok, button pressed, go to sleep!
		printf("going to sleep at: ");
		print_date();
		led.off();
		ksleep();
		pressed = false;
		printf("woke up at.. ");
		print_date();
		kwkup_stop();
	}
		
	return 0;
}


template <>
void interrupt::handler<interrupt::irq::RTC_WKUP>() {
	// Order is critical here.  Reversing these will give endless IRQs!
	EXTI->PR1 = (1<<19); // clear rtc from exti pending
	RTC->ISR = ~(1<<10); // Clear WUTF (
	ITM->stim_blocking(0, '$');
}

template <>
void interrupt::handler<interrupt::irq::EXTI1>() {
	EXTI->PR1 |= (1<<sw_3.n);
	ITM->stim_blocking(0, '!');
	pressed = true;
}

