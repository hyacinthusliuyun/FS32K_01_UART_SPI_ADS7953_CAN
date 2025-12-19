#ifndef __USERINIT_H_
#define __USERINIT_H_

#include "Cpu.h"
#include "S32K148.h"
#include "pins_driver.h"
#include "pin_mux.h"
#include "clockMan1.h"
#include "osif.h"


#include "USERINIT.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <status.h>
#include <math.h>

#include "adc_driver.h"
#include "uart_pal1.h"
//#include "spi2.h"
#include "lpspi_master_driver.h"

#include "MLYTASK.h"
#include "led.h"
#include "key.h"
#include "MYCAN.H"
#include "MYADC.h"
//#include "LiuyunSPI.h"
#include "LiuyunLPSPI2.h"


#define TIMEOUT 200UL
void USERINIT(void);
void MLY_UART1_SEND(const char *fmt, ...);
void CAN1_Init(void);

extern uint8_t txData[8] ;
extern uint8_t DEBUG_MSG[128];

#endif

