#ifndef __USERINIT_H_
#define __USERINIT_H_

#include "Cpu.h"
#include "S32K148.h"
#include "pins_driver.h"
#include "pin_mux.h"
#include "clockMan1.h"
#include "osif.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <status.h>
#include <math.h>

#include "adc_driver.h"
#include "uart_pal1.h"
#include "lpspi_master_driver.h"

#include "MLYTASK.h"
#include "led.h"
#include "LiuyunLPSPI2.h"
#include "JustFloatUART.h"

#define TIMEOUT 200UL
void USERINIT(void);
void MLY_UART1_SEND(const char *fmt, ...);

extern uint8_t txData[8] ;
extern uint8_t DEBUG_MSG[128];

#endif

