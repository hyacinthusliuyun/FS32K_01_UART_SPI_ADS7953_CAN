#include "JustFloatUART.h"
#include "USERINIT.h"

#define JFLOAT_CH_COUNT      16U
#define JFLOAT_TAIL_SIZE     4U
#define JFLOAT_VALID_SIZE    2U
#define JFLOAT_FRAME_SIZE    (JFLOAT_CH_COUNT * sizeof(float) + JFLOAT_VALID_SIZE + JFLOAT_TAIL_SIZE)

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
    uint16_t validMap = 0U;

    if ((raw == 0) || (valid == 0))
    {
        return;
    }

    for (uint8_t i = 0U; i < JFLOAT_CH_COUNT; ++i)
    {
        float val = (float)raw[i];
        uint8_t fbytes[sizeof(float)] = {0};

        if (valid[i] != 0U)
        {
            validMap |= (uint16_t)(1U << i);
        }

        memcpy(fbytes, &val, sizeof(float));
        memcpy(&frame[pos], fbytes, sizeof(float));
        pos += (uint16_t)sizeof(float);
    }

    frame[pos++] = (uint8_t)(validMap & 0xFFU);
    frame[pos++] = (uint8_t)(validMap >> 8);

    memcpy(&frame[pos], kJustFloatTail, JFLOAT_TAIL_SIZE);
    pos += JFLOAT_TAIL_SIZE;

    JFLOAT_UART_SendBytes(frame, pos);
}
