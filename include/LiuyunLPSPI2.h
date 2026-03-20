#ifndef LIUYUNLPSPI2_H_
#define LIUYUNLPSPI2_H_

#include <stdint.h>

extern uint32_t ADS7953_GetValueOrig[16];
extern uint32_t ADS7953_SpiErrCnt;
extern uint32_t ADS7953_SpiOkCnt;
extern uint32_t ADS7953_TagErrCnt;

void ADS7953_Scan(void);
void ADS7953_ScanAll_Manual(void);
void ADS7953_CheckChannel(uint8_t ch);

#endif