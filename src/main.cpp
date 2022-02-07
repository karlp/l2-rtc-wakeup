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
#include <usb/usb.h>
#include <usb/descriptor.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#if defined (STM32WB)
auto led_r = GPIOB[1];
auto led_g = GPIOB[0];
auto led_b = GPIOB[5];
#define HAS_LED_BLUE 1
#elif defined(STM32F4)
// Nucleo 144 boards at least...
auto led_r = GPIOB[14];
auto led_g = GPIOB[0];
auto led_b = GPIOB[7];
#elif defined(STM32F3)
// f3 disco
auto led_r = GPIOE[9];
auto led_g = GPIOE[15];
auto led_b = GPIOE[12];
#elif defined(STM32L4)
auto led_g = GPIOA[5];
auto sw_1 = GPIOC[13];
#else
#error "Platform unsupported"
#endif

/* disable this with a debugger if you want to measure power sanely
 * You'll then have to watch the ITM channels to see that things do what you think.
 */
bool opt_really_use_leds = false;

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

static void kexti_init(void) {
	// sw_3.. is D1...
#if defined(STM32WB)
	RCC.enable(rcc::GPIOD);
#elif defined(STM32L4)
	RCC.enable(rcc::GPIOC);
#else
#error "Unhandled platform, check your switch/led ports"
#endif

	sw_1.set_mode(Pin::Mode::Input);
	sw_1.set_pull(Pin::Pull::PullUp);

	SYSCFG->EXTICR[sw_1.n >> 2] &= ~(0xf << ((sw_1.n & 0x3) * 4));
	SYSCFG->EXTICR[sw_1.n >> 2] |= (sw_1.get_portnum() << ((sw_1.n & 0x3) * 4));

	EXTI->IMR1 |= (1 << sw_1.n); // unmask
	EXTI->FTSR1 |= (1 << sw_1.n); // falling edge

#if defined(STM32WB)
	NVIC.enable(interrupt::irq::EXTI1);
#elif defined(STM32L4)
	NVIC.enable(interrupt::irq::EXTI15_10);
#endif
}

/**
 * Green uses a "normal" busy task, blocking loop of it's own.
 * @param pvParameters
 */
static void prvTaskBlinkGreen(void *pvParameters)
{
	(void)pvParameters;
	if (opt_really_use_leds) {
#if defined(STM32L4)
		RCC.enable(rcc::GPIOA);
#elif defined(STM32WB)
		RCC.enable(rcc::GPIOB);
#else
#warning "no leds configured..."
#endif
		led_g.set_mode(Pin::Output);
	}

	int i = 0;
	while (1) {
		i++;
		vTaskDelay(pdMS_TO_TICKS(15000));
	        ITM->stim_blocking(1, (uint8_t)('a' + (i%26)));
		if (opt_really_use_leds) {
			led_g.toggle();
		}
		printf("G: %d\n", i);
	}
}

#if defined(HAS_LED_BLUE)
static TimerHandle_t xBlueTimer;
static TaskHandle_t th_blue;
static void prvTimerBlue(TimerHandle_t xTimer)
{
	xTaskNotifyGive(th_blue);
}
/**
 * Blue uses a timer to notify the task it's time to do the blink.
 * @param pvParameters
 */
static void prvTaskBlinkBlue(void *pvParameters) {
	(void)pvParameters;
	led_b.set_mode(Pin::Output);
	
	int i = 0;
	while (1) {
		i++;
		xTaskNotifyWait(1, 1, NULL, portMAX_DELAY);
		ITM->stim_blocking(2, (uint8_t)('A' + (i%26)));
		if (opt_really_use_leds) {
			led_b.toggle();
		}
		printf("B: %d\n", i);
	}
}
#endif

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

static void print_date(void) {
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
		(1000*(255 - s)/256) // based on "default" fck_spre
		);
}

static TaskHandle_t th_button;
void prvTaskButton(void *pvParameters) {
	(void)pvParameters;

	int i = 0;
	while (1) {
		i++;
		BaseType_t rc = xTaskNotifyWait(1, 1, NULL, pdMS_TO_TICKS(10000));
		if (rc) {
			printf("Woke via button press!\n");
		}
		print_date();
	}
}


int main() {
	krcc_init32();
	klpuart_init();
	printf("Started from scratch, clocked up\n");

	// Turn on DWT_CYCNT.  We'll use it ourselves, and pc sampling needs it too.
	DWT->CTRL |= 1;

	krtc_init();
	printf("RTC init, ready to  start!\n");

	kexti_init();

	xTaskCreate(prvTaskBlinkGreen, "blink.green", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
#if defined(HAS_LED_BLUE)
	xTaskCreate(prvTaskBlinkBlue, "blink.blue", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &th_blue);

	xBlueTimer = xTimerCreate("blue.blink", 200 * portTICK_PERIOD_MS, true, 0, prvTimerBlue);
	if (xBlueTimer) {
		if (xTimerStart(xBlueTimer, 0) != pdTRUE) {
			/* whee */
		} else {
			// boooo
		}
	} else {
		// boooo!!!!! fixme trace?
	}
#endif

	xTaskCreate(prvTaskButton, "buttons", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &th_button);

	// Required to use FreeRTOS ISR methods!
	// ( set the priority for all of them, regardless of platform...
	NVIC.set_priority(interrupt::irq::EXTI1, 6<<configPRIO_BITS);
	NVIC.set_priority(interrupt::irq::EXTI15_10, 6<<configPRIO_BITS);

	vTaskStartScheduler();

	return 0;
}

// TODO -figure out how to give this to freertosconfig?
//#define vPortSVCHandler SVC_Handler
//#define xPortPendSVHandler PendSV_Handler
//#define xPortSysTickHandler SysTick_Handler
extern "C" {
	void vPortSVCHandler(void);
	void xPortPendSVHandler(void);
//	void xPortSysTickHandler(void);
	void LPTIM1_IRQHandler(void);
}
template <>
void interrupt::handler<interrupt::exception::SVCall>() {
	vPortSVCHandler();
}
template <>
void interrupt::handler<interrupt::exception::PendSV>() {
	xPortPendSVHandler();
}
//template <>
//void interrupt::handler<interrupt::exception::SysTick>() {
//	xPortSysTickHandler();
//}
extern void LPTIM_IRQHandler(void);
template <>
void interrupt::handler<interrupt::irq::LPTIM1>() {
	LPTIM_IRQHandler();
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
	EXTI->PR1 |= (1<<sw_1.n);
	vTaskNotifyGiveFromISR(th_button, NULL);
}


