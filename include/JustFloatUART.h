#ifndef JUSTFLOATUART_H_
#define JUSTFLOATUART_H_

#include <stdint.h>

/* 0: 保留原ASCII输出；1: 使用JustFloat二进制输出 */
#define ADS7953_TX_USE_JUSTFLOAT 1U

void JFLOAT_UART_SendBytes(const uint8_t *buf, uint16_t len);
void JFLOAT_UART_SendAds16RawAsFloat(const uint32_t raw[16], const uint8_t valid[16]);

#endif