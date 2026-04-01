#ifndef __MYCAN_H_
#define __MYCAN_H_

// #define USE_CAN 0

//#if USE_CAN
#include "Cpu.h"
#include "USERINIT.h"
#include "can_pal1.h"
#include "can_pal.h"
extern volatile bool rxComplete;
extern can_message_t rxMsg;      // �յ��Ķ���

/* �������� */
void APP_CAN_Init(void);
// status_t APP_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len);
void APP_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len);
/* ���� S32DS ���ɵ�ʵ������ MYCAN.c ʹ�� */
void APP_CAN_LoopbackTest(void);
#define RX_MAILBOX  0UL
#define TX_MAILBOX  1UL
#define Rx_Filter 0x123u
#define RX_MASK  0x7FFu
#endif
