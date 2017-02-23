#include "lpc_types.h"
#include "LPC17xx.h"
#include "lpc17xx_gpio.h"
#include "ff_test_term.h"
#include "diskio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_rtc.h"
#include "rtc_cal.h"
#include "lpc17xx_nvic.h"

void LED1_on(void);
void LED1_off(void);
void init_in_main(void);
void startup_delay(void);
