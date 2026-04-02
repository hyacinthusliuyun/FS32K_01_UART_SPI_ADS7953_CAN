#ifndef LIUYUNLPSPI2_H_
#define LIUYUNLPSPI2_H_

#include <stdint.h>

void ADS7953_Scan(void);
void ADS7953_Scan_channel(uint8_t ch);
void ADS7953_ScanAll_Manual(void);

#endif