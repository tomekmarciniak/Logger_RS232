#include "init.h"
#define LED1_PORT  0
#define LED1_PIN   20
#define LED1_MASK (1 << LED1_PIN)

volatile uint32_t SysTickCnt;      /* SysTick Counter                    */

void LED1_off(void)
{
	GPIO_ClearValue(LED1_PORT, LED1_MASK);
}

void LED1_on(void)
{
	GPIO_SetValue(LED1_PORT, LED1_MASK);
}

/* SysTick Interrupt Handler (1ms) */
void SysTick_Handler (void)
{
	static uint32_t flip, led_prescale;
	static uint32_t div10;

	if (++led_prescale > 500) {
		led_prescale = 0;
		if (flip) { LED1_on();  }
		else      { LED1_off(); }
		flip = !flip;
	}

	if (++div10 >= 10) {
		div10 = 0;
		disk_timerproc(); /* Disk timer function (100Hz) */
	}

	ff_test_term_timer_callback();

	SysTickCnt++;
}

void init_in_main(void)
{
	// DeInit NVIC and SCBNVIC
	//NVIC_DeInit();
	//NVIC_SCBDeInit();

	/* Configure the NVIC Preemption Priority Bits:
	 * two (2) bits of preemption priority, six (6) bits of sub-priority.
	 * Since the Number of Bits used for Priority Levels is five (5), so the
	 * actual bit number of sub-priority is three (3)
	 */
	NVIC_SetPriorityGrouping(0x05);

	//  Set Vector table offset value
#if (__RAM_MODE__==1)
	NVIC_SetVTOR(0x10000000);
#else
	NVIC_SetVTOR(0x00000000);
#endif

	RTC_TIME_Type initial_time = {
		0, 55, 11, // SS:MM:HH
		15, 0, 0,  // DOM, DOW, DOY
		7, 2010    // MONTH, YEAR
	};

	/* LED(s) */
	PINSEL_CFG_Type PinCfg;

	PinCfg.Funcnum   = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode   = 0;
	PinCfg.Pinnum    = LED1_PIN;
	PinCfg.Portnum   = LED1_PORT;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, LED1_MASK, 1);

	/* RTC */
	rtc_cal_init(&initial_time);

	SysTick_Config(SystemCoreClock/10000 - 1); /* Generate interrupt each 1 ms   */
}

void startup_delay(void)
{
	volatile unsigned long i;
	for (i = 0; i < 500000; i++) { ; }
}
