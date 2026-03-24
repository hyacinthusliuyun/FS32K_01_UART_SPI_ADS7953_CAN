#include "JustFloatUART.h"
#include "USERINIT.h"

#define JFLOAT_CH_COUNT      16U
#define JFLOAT_TAIL_SIZE     4U
#define JFLOAT_FRAME_SIZE    (JFLOAT_CH_COUNT * sizeof(float) + JFLOAT_TAIL_SIZE)  /* 16*4+4 = 68 bytes */

static const uint8_t kJustFloatTail[JFLOAT_TAIL_SIZE] = {0x00U, 0x00U, 0x80U, 0x7FU};

//uart_pal1
void JFLOAT_UART_SendBytes(const uint8_t *buf, uint16_t len)
{
    if ((buf == 0) || (len == 0U))
    {
        return;
    }

    (void)UART_SendDataBlocking(&uart_pal1_instance, (const uint8_t *)buf, len, TIMEOUT);
}

void JFLOAT_UART_SendAds16RawAsFloat(const uint32_t raw[16], const uint8_t valid[16])
{
    uint8_t frame[JFLOAT_FRAME_SIZE] = {0};
    uint16_t pos = 0U;

    if ((raw == 0) || (valid == 0))
    {
        return;
    }

    /* 直接发送16个float（小端），无任何额外字节 */
    for (uint8_t i = 0U; i < JFLOAT_CH_COUNT; ++i)
    {
        float val = (float)raw[i];
        memcpy(&frame[pos], &val, sizeof(float));
        pos += sizeof(float);
    }

    /* 发送标准JustFloat帧尾 */
    memcpy(&frame[pos], kJustFloatTail, JFLOAT_TAIL_SIZE);

    JFLOAT_UART_SendBytes(frame, JFLOAT_FRAME_SIZE);
}
