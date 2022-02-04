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
#include <uart/uart.h>

#if defined(STM32WB)
auto led_r = GPIOB[1];
auto led_g = GPIOB[0];
auto led_b = GPIOB[5];
auto led = led_g;
auto sw_1 = GPIOC[4];
auto sw_2 = GPIOD[0];
auto sw_3 = GPIOD[1];
auto exti_rtc_wkup = 19;
#elif defined(STM32L4)
auto led_b = GPIOA[5];
auto sw_3 = GPIOC[13];
auto led = led_b;
auto exti_rtc_wkup = 20;
#else
#error "Unhandled platform"
#endif

/* disable this with a debugger if you want to measure power sanely
 * You'll then have to watch the ITM channels to see that things do what you think.
 */
bool opt_really_use_leds = true;

#include <cerrno>
#include <cstdlib>
#include <unistd.h>


extern "C" int _write(int file, char* ptr, int len) {
        int i;

        if (file == STDOUT_FILENO || file == STDERR_FILENO) {
                for (i = 0; i < len; i++) {
                        if (ptr[i] == '\n') {
				LPUART1.write_blocking('\r');
                        }
		LPUART1.write_blocking(ptr[i]);
                }
                return i;
        }
        errno = EIO;
        return -1;
}


static inline uint32_t __get_PRIMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) :: "memory");
  return(result);
}

static inline void __set_PRIMASK(uint32_t priMask)
{
  __asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}

static inline void __disable_irq(void)
{
  __asm volatile ("cpsid i" : : : "memory");
}





/**
 * Run at 32Mhz instead of laks "top speed" by default.
 * 32Mhz is what many ST WB demos run at, and if you don't need more, it's
 * probably a good choice.
 */
#if defined(STM32WB)
void krcc_init32(void) {
	// Prefetch and both caches, plus 1WS for 32MHz
	FLASH->ACR = 0x700 | 1;

	// Enable HSE.
	RCC->CR |= (1<<16);
	while(!(RCC->CR & (1<<17)));
	
	/* No PLL, that just wastes power! */

	// Make sure to clear bits, we may be waking up!
	RCC->CFGR &= ~0x3;
	RCC->CFGR |= 0x2;
	while ((RCC->CFGR & (3<<2)) != (2<<2)); // SWS = HSE
	// Leave prescalers alone...
}
#endif

#if defined(STM32L4)
static void krcc_init32(void) {
	// We're just goign to use MSI at 32Mhz, as it's "easy" to setup,
	// and it doesn't matter what your crystal is.  (Even if nucleos have 8Mhz)


	// Prefetch and both caches, plus 1WS for 32MHz
	FLASH->ACR = 0x700 | 1;

	// Enable MSI (it should already be on, it's the reset clock)
	RCC->CR |= (1<<3) | 1; // Also enable range selection here
	// Can't modify MSI range when it's on and not ready
	while (!(RCC->CR & (1 << 1)));
	uint32_t msi_range = 10; // 32Mhz
	RCC->CR &= ~(0xf<<4);
	RCC->CR |= (msi_range << 4);

	RCC->CFGR &= ~(0x3<<0); // Set MSI as clock source
	// L4 Needs these to be enabled first.
	RCC.enable(rcc::PWR);
	RCC.enable(rcc::SYSCFG);
}
#endif



static void krtc_init(void)
{
	// always need that even if RTC itself is already setup
	PWR->CR1 |= (1 << 8); // Unlock backup domain

	// Use the last backup register as a flag that we're already initialized
	if (RTC->BKP[19] == 0xcafe) {
		printf("RTC: already setup!\n");
		return;
	}

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
	// WB simply does a single read here, and L4 just hangs
//	while (!(RCC->APB1ENR1 & rcc::RTCAPB)) {
//		; // make sure we have access!
//	}

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

	RTC->BKP[19] = 0xcafe;
}


static void kwkup_stop(void) {
	RTC.unlock();
	RTC->CR &= ~(1<<10); // Disable WUTE
	RTC.lock();
}

static void kwkup_start(uint16_t wu_ticks)
{
	// Set it up now, but we will turn it on later.
	// RTC Wakeup is via exti 19! only listed in the nvic section
	// l4 says rtc wakeup is exti line 20!  we know it's 19 on wb
	EXTI->RTSR1 |= (1 << exti_rtc_wkup);
	EXTI->IMR1 |= (1 << exti_rtc_wkup);
	// we want to wakeup, but we don't need a handler, but I can't get it to work without?
	NVIC.enable(interrupt::irq::RTC_WKUP);


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
#if defined(STM32WB)
	RCC.enable(rcc::GPIOD);
#elif defined(STM32L4)
	RCC.enable(rcc::GPIOC);
#else
#error "Unhandled platform, check your switch/led ports"
#endif

	sw_3.set_mode(Pin::Mode::Input);
	sw_3.set_pull(Pin::Pull::PullUp);
	
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

// Used for just plain sleep
class HandleNull {
public:
	void pre(void) {};
	void post(void) {};
};

/**
 * Can actually handle stop modes 0,1,2, standby, and stop.
 */
class HandleStop {
private:
	uint32_t _mode;
	uint32_t _rcc_cr, _rcc_cfgr;
public:
	HandleStop(uint32_t mode): _mode{mode} {};

	void pre(void) {
//		printf("stop pre %lu\n", _mode);
		_rcc_cr = RCC->CR;
		_rcc_cfgr = RCC->CFGR;
		PWR.set_lpms(_mode);
//		PWR.set_lpms_c2(_mode);
//		if ((PWR->EXTSCR >> 8) & 0xf) {
//			// if any of thos fucking flags is set, clear them all!
//			PWR->EXTSCR |= 0x3;
//		}
		// Enable SLEEPDEEP
		SCB->SCR |= (1<<2);
	}
	void post(void) {
//		printf("stop post\n");
		// 3. We wakeup with either HSI16 or MSI depending on RCC->CFGR:STOPWUCK...
		// we're just going to to with "restore RCC->CR and RCC-CFGR....
		RCC->CR = _rcc_cr;
		RCC->CFGR = _rcc_cfgr;
	}
};

/**
 * Handle clocking down to allow lprun / lp sleep
 */
class HandleLP {
private:
	uint32_t _msi_range;
	uint32_t _rcc_cr, _rcc_cfgr;
public:
	HandleLP(uint32_t msi_range): _msi_range{msi_range} {};

	void pre(void) {
		_rcc_cr = RCC->CR;
		_rcc_cfgr = RCC->CFGR;
		// Ensure MSI is on, and switch down to that range...
		RCC->CR |= (1<<3) | 1; // Also enable range selection here
		// Can't modify MSI range when it's on and not ready
		while (!(RCC->CR & (1 << 1)));
		RCC->CR &= ~(0xf<<4);
		RCC->CR |= (_msi_range << 4);

		RCC->CFGR &= ~(0x3<<0); // Set MSI as clock source
		// SLEEPDEEP must be clear
		SCB->SCR &= ~(1<<2);

		PWR->CR1 |= (1<<14); // Set regulator low power mode
	}

	void post(void) {
		// Restore regulator power
		PWR->CR1 &= ~(1<<14);
		while (PWR->SR2 & (1<<9)); // Wait for REGLPF

		// restore system clock settings
		RCC->CR = _rcc_cr;
		RCC->CFGR = _rcc_cfgr;
	}
};

// for starters...
auto handler = HandleStop(2);
//auto handler = HandleNull();
//auto handler = HandleLP(5); // 0=100kHz, 1=200, 2=400,3=800,4=1M, 5=2M

static void ksleep2(void) {
	kwkup_start(14000);

	uint32_t primask_bit = __get_PRIMASK( );
	__disable_irq( );
	handler.pre();
	__asm volatile ("wfi");
	handler.post();
	__set_PRIMASK(primask_bit);

	kwkup_stop();
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

static void klpuart_init(void) {
//	RCC.enable(rcc::USART1);
	RCC.enable(rcc::LPUART1);

	// sysclock? lpuart1
	RCC->CCIPR &= ~(0x3<<10);
	RCC->CCIPR |= (1<<10);
	// usart1
	RCC->CCIPR &= ~(0x3<<0);
	RCC->CCIPR |= (1<<0);


#if defined(STM32WB)
	// PA2,3 for lpuart1
	RCC.enable(rcc::GPIOA);
	auto lp1_rx = GPIOA[2];
	auto lp1_tx = GPIOA[3];
#elif defined(STM32L4)
	RCC.enable(rcc::GPIOC);
	auto lp1_rx = GPIOC[0];
	auto lp1_tx = GPIOC[1];
#endif
	lp1_rx.set_af(8);
	lp1_tx.set_af(8);
	lp1_rx.set_mode(Pin::Mode::AF);
	lp1_tx.set_mode(Pin::Mode::AF);

//	RCC.enable(rcc::GPIOB);
//	auto pb6 = GPIOB[6];
//	auto pb7 = GPIOB[7];
//	pb6.set_af(7);
//	pb7.set_af(7);
//	pb6.set_mode(Pin::Mode::AF);
//	pb7.set_mode(Pin::Mode::AF);
//
//	USART1->CR1 = (1<<29); // fifo, 8bit, no parity, no special
//	//USART1->CR1 = 0;
//	USART1->CR2 = 0; // standard
//	// this is for lpuart USART1->BRR = (256*32000000u / 115200); // 71111?
//	USART1->BRR = 32000000u / 115200u;
//	USART1->CR1 |= (1<<3)|1; // TX only, enabled

	LPUART1->CR1 = (1<<29); // fifo, 8bit, no parity, no special
	//LPUART1->CR1 = 0;
	LPUART1->CR2 = 0; // standard
	LPUART1->BRR = (32000000u / 115200) * 256; // careful with overflows!
	LPUART1->CR1 |= (1<<3)|1; // TX only, enabled

}

static volatile bool pressed = false;

int main() {
	krcc_init32();

	klpuart_init();
	printf("Started from scratch, clocked up\n");

	// Turn on DWT_CYCNT.  We'll use it ourselves, and pc sampling needs it too.
	DWT->CTRL |= 1;
	
	krtc_init();
	printf("RTC init, ready to  start!\n");
	
	kexti_init();
	

	if (opt_really_use_leds) {
		RCC.enable(rcc::GPIOA);
		RCC.enable(rcc::GPIOB);
		led.set_mode(Pin::Output);
	}

	// Depending on sleep mode, you may be restarting from scratch, or looping
	uint8_t k = 0;
	while (1) {
		printf("start loop\n");
		while (!pressed) {
			for (int i = 0; i < 0x80000; i++) {
				asm volatile ("nop");
			}
			if (opt_really_use_leds) {
				led.toggle();
			}
//			USART1.write_blocking('a' + k++);
			LPUART1.write_blocking('a' + k++);
			if (k >= 26) {
				k = 0;
//				USART1.write_blocking('\n');
				LPUART1.write_blocking('\n');
				print_date();
			}
		}
		// Ok, button pressed, go to sleep!
		printf("going to sleep at: ");
		print_date();
		led.off();

		ksleep2();
		pressed = false;
		printf("woke up at.. ");
		print_date();
	}
		
	return 0;
}


// We shouldn't need an irq, we're just using it as a wakeup event!
template <>
void interrupt::handler<interrupt::irq::RTC_WKUP>() {
	// Order is critical here.  Reversing these will give endless IRQs!
	EXTI->PR1 = (1<<exti_rtc_wkup); // clear rtc from exti pending
	RTC->ISR = ~(1<<10); // Clear WUTF (
	//ITM->stim_blocking(0, '$');
	LPUART1->TDR = '$';
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
	EXTI->PR1 |= (1<<sw_3.n);
//	ITM->stim_blocking(0, '!');
	pressed = true;
}
