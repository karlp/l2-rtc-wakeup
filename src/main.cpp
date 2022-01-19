#include <cstdio>

#include <cal/cal.h>
#include <cortex_m/debug.h>
#include <dma/dma.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <pwr/pwr.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
#include <rtc/rtc.h>
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
#endif

/* disable this with a debugger if you want to measure power sanely
 * You'll then have to watch the ITM channels to see that things do what you think.
 */
bool opt_really_use_leds = false;
/**
 * Run at 32Mhz instead of laks "top speed" by default.
 * 32Mhz is what many ST WB demos run at, and if you don't need more, it's
 * probably a good choice.
 */
const bool opt_clock_32mhz = true;

void krcc_init32(void) {
	// Prefetch and both caches, plus 1WS for 32MHz
	FLASH->ACR = 0x700 | 1;

	// Enable HSE.
	RCC->CR |= (1<<16);
	while(!(RCC->CR & (1<<17)));

	/* I initially setup PLL to 32Mhz, because I had PLL to 64, and just downtuned
	 * but if you want lower power, use HSE by itself at 32.  The PLL is useless burn
	 */
#if defined(USE_PLL_32)
	// Configure and enable PLL.
	// R=2, Q = 2, P = 2, M = 2, N = 8, src=HSE
	const auto m = 2;
	const auto n = 8;
	const auto p = 2;
	const auto q = 2;
	const auto r = 4;
	//const auto hse_pre_div2 = false;

	RCC->PLLCFGR = ((r-1)<<29) | (1<<28) | ((q-1)<<25) | ((p-1)<<17) | (n<<8) | ((m-1)<<4) | (3<<0);
	RCC->CR |= (1<<24);
	while(!(RCC->CR & (1<<25)));

	// Switch to PLL.
	RCC->CFGR |= 0x3;
	while((RCC->CFGR & (3 << 2)) != (3 << 2)); // SWS = PLL
#else
	//RCC->CFGR &= ~0x3; // do I need this? unlikely at reset? possibly at other times?
	RCC->CFGR |= 0x2;
	while ((RCC->CFGR & (2<<2)) != (2<<2)); // SWS = HSE
#endif
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

static TimerHandle_t xBlueTimer;
static TaskHandle_t th_blue;
static void prvTimerBlue(TimerHandle_t xTimer)
{
	xTaskNotifyGive(th_blue);
}

/**
 * Green uses a "normal" busy task, blocking loop of it's own.
 * @param pvParameters
 */
static void prvTaskBlinkGreen(void *pvParameters)
{
	(void)pvParameters;
	led_g.set_mode(Pin::Output);

	int i = 0;
	while (1) {
		i++;
		vTaskDelay(pdMS_TO_TICKS(500));
	        ITM->stim_blocking(1, (uint8_t)('a' + (i%26)));
		if (opt_really_use_leds) {
			led_g.toggle();
		}
		printf("G: %d\n", i);
	}
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


int main() {
	if (opt_clock_32mhz) {
		krcc_init32();
	} else {		
		rcc_init();
	}
	printf("clocked up, turning on rtc...\n");

	// Turn on DWT_CYCNT.  We'll use it ourselves, and pc sampling needs it too.
	DWT->CTRL |= 1;
	
	krtc_init();
	printf("RTC init, ready to start!\n");

	RCC.enable(rcc::GPIOB);
	RCC.enable(rcc::GPIOE);

	xTaskCreate(prvTaskBlinkGreen, "blink.green", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
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

	// Required to use FreeRTOS ISR methods!
//	NVIC.set_priority(interrupt::irq::DMA1_CH1, 6<<configPRIO_BITS);

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
	void xPortSysTickHandler(void);
}
template <>
void interrupt::handler<interrupt::exception::SVCall>() {
	vPortSVCHandler();
}
template <>
void interrupt::handler<interrupt::exception::PendSV>() {
	xPortPendSVHandler();
}
template <>
void interrupt::handler<interrupt::exception::SysTick>() {
	xPortSysTickHandler();
}

