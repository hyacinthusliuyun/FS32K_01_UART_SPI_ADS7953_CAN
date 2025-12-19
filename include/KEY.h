#ifndef __KEY_H_
#define __KEY_H_

// ∂®“ÂDEFINE KEY1, KEY2, KEY3 PIN & PINPORT
#define KEY1_PIN_PORT    PTA
#define KEY1_PIN          25U

#define KEY2_PIN_PORT    PTE
#define KEY2_PIN          13U


#include "CPU.h"
#include "s32k148.h"
#include "KEY.h"
uint8_t KEY_READ(void);
#endif

