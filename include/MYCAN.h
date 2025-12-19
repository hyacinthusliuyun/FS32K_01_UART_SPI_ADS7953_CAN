#ifndef __MYCAN_H_
#define __MYCAN_H_

#include "Cpu.h"
#include "USERINIT.h"
#include "can_pal1.h"
#include "can_pal.h"
extern volatile bool rxComplete;
extern can_message_t rxMsg;      // 收到的东西

/* 函数声明 */
void APP_CAN_Init(void);
// status_t APP_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len);
void APP_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len);
/* 声明 S32DS 生成的实例，供 MYCAN.c 使用 */
void APP_CAN_LoopbackTest(void);
#define RX_MAILBOX  0UL
#define TX_MAILBOX  1UL
#define Rx_Filter 0x123u
#define RX_MASK  0x7FFu
#endif
