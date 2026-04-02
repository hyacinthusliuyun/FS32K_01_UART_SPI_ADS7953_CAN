#include "USERINIT.h"
#include "LiuyunLPSPI2.h"
#include "JustFloatUART.h"
#include "osif.h"
#include <stdio.h>
#include <string.h>

/*默认为justfloat协议ADS7953_TX_USE_JUSTFLOAT*/
#define ADS7953_OUTPUT_USE_JUSTFLOAT  ADS7953_TX_USE_JUSTFLOAT

#define ADS7953_CH_COUNT             16U
#define ADS7953_MODE_MANUAL          0x1U
#define ADS7953_ENABLE_PROG          (1U << 11)
#define ADS7953_CH_SHIFT             7U
#define ADS7953_RANGE_2VREF          1U

/* 软件控制 CS (PTE10) - 用户自定义GPIO */
#define ADS7953_CS_PORT              PTE
#define ADS7953_CS_PIN               10U

static uint16_t ADS7953_Transfer16(uint16_t tx);

/* Manual模式命令字（参考例程STM32）:
 * [15:12] = 0001 手动模式
 * [11]    = ENABLE_PROG（使能编程）
 * [9:7]   = 通道号
 * [6]     = RANGE（0=0~VREF, 1=0~2*VREF）
 */
static inline uint16_t ADS7953_CmdManual(uint8_t ch)
{
    return (uint16_t)(((uint16_t)ADS7953_MODE_MANUAL << 12) |
                     ADS7953_ENABLE_PROG |
                     ((uint16_t)(ch & 0x0FU) << ADS7953_CH_SHIFT) |
                     ((uint16_t)ADS7953_RANGE_2VREF << 6));
}

/* 软件控制 CS 的 16 位传输（参考例程 STM32）
 * 注意：CS=HIGH 后添加延时，满足 ADS7953 t_q 要求（>40ns）
 */
static uint16_t ADS7953_Transfer16(uint16_t tx)
{
    uint16_t rx = 0;
    uint8_t tx_buf[2] = {(uint8_t)(tx >> 8), (uint8_t)(tx & 0xFFU)};
    uint8_t rx_buf[2] = {0};

    /* CS LOW */
    PINS_DRV_WritePin(ADS7953_CS_PORT, ADS7953_CS_PIN, 0);

    /* SPI 传输 */
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx_buf, rx_buf, 2U, 10U);

    /* CS HIGH */
    PINS_DRV_WritePin(ADS7953_CS_PORT, ADS7953_CS_PIN, 1);

    /* 延时满足 ADS7953 t_q 要求（>40ns），此处约 2µs */
    for (volatile uint32_t delay = 100U; delay--; );

    rx = (uint16_t)((((uint16_t)rx_buf[0]) << 8) | rx_buf[1]);

    return rx;
}

/* 电压转换函数（参考STM32工作项目） */
static float ADS7953_RawToOffsetVolt(uint16_t raw)
{
    return (float)raw * (3.3f / 4095.0f);
}

static float ADS7953_RawToRealVolt(uint16_t raw)
{
    float real_v = ((float)raw * (10.0f / 4095.0f)) - 5.0f;
    if (real_v > 5.0f) real_v = 5.0f;
    else if (real_v < -5.0f) real_v = -5.0f;
    return real_v;
}

/* Manual模式整帧扫描（参考 STM32 工作项目）:
 * - 4 个 warmup 帧
 * - 16 个通道扫描
 * - 直接存储 raw 值
 */
void ADS7953_ScanAll_Manual(void)
{
    uint16_t raw[ADS7953_CH_COUNT];

    /* 清零 */
    for (uint8_t i = 0U; i < ADS7953_CH_COUNT; ++i)
    {
        raw[i] = 0U;
    }

    /* Step 1: 4 个 warmup 帧 */
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        (void)ADS7953_Transfer16(ADS7953_CmdManual((uint8_t)(i & 0x0FU)));
    }

    /* Step 2: 扫描 16 个通道 */
    for (uint8_t ch = 0U; ch < ADS7953_CH_COUNT; ++ch)
    {
        uint16_t frame = ADS7953_Transfer16(ADS7953_CmdManual(ch));
        raw[ch] = (uint16_t)(frame & 0x0FFFU);
    }

#if ADS7953_OUTPUT_USE_JUSTFLOAT
    /* JUSTFLOAT 二进制输出 */
    {
        float vals[ADS7953_CH_COUNT];
        for (uint8_t ch = 0U; ch < ADS7953_CH_COUNT; ++ch)
        {
            vals[ch] = (float)raw[ch];
        }
        JFLOAT_UART_SendFloatArray16(vals);
    }
#else
    /* ASCII 文本输出（参考 STM32 工作项目格式） */
    char line[72];
    int len;

    for (uint8_t ch = 0U; ch < ADS7953_CH_COUNT; ++ch)
    {
        float or_v = ADS7953_RawToOffsetVolt(raw[ch]);
        float real_v = ADS7953_RawToRealVolt(raw[ch]);
        len = snprintf(line, sizeof(line), "ch%u,or=%.3f,or_v=%.3f,real_v=%.3f\r\n",
                       (unsigned int)ch, (double)raw[ch], (double)or_v, (double)real_v);
        if (len > 0)
        {
            MLY_UART1_SEND("%s", line);
        }
    }
#endif
}

/* 单通道扫描 */
void ADS7953_Scan_channel(uint8_t ch)
{
    uint16_t raw[ADS7953_CH_COUNT];

    if (ch >= ADS7953_CH_COUNT)
    {
        MLY_UART1_SEND("ADS7953 ch invalid: %d\r\n", ch);
        return;
    }

    /* 清零 */
    for (uint8_t i = 0U; i < ADS7953_CH_COUNT; ++i)
    {
        raw[i] = 0U;
    }

    /* Step 1: 4 个 warmup 帧 */
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        (void)ADS7953_Transfer16(ADS7953_CmdManual((uint8_t)(i & 0x0FU)));
    }

    /* Step 2: 扫描目标通道 */
    uint16_t frame = ADS7953_Transfer16(ADS7953_CmdManual(ch));
    raw[ch] = (uint16_t)(frame & 0x0FFFU);

#if ADS7953_OUTPUT_USE_JUSTFLOAT
    /* JUSTFLOAT 二进制输出 */
    {
        float vals[ADS7953_CH_COUNT];
        for (uint8_t i = 0U; i < ADS7953_CH_COUNT; ++i)
        {
            vals[i] = (float)raw[i];
        }
        JFLOAT_UART_SendFloatArray16(vals);
    }
#else
    /* ASCII 文本输出 */
    {
        char line[72];
        int len;
        float or_v = ADS7953_RawToOffsetVolt(raw[ch]);
        float real_v = ADS7953_RawToRealVolt(raw[ch]);
        len = snprintf(line, sizeof(line), "ch%u,or=%.3f,or_v=%.3f,real_v=%.3f\r\n",
                       (unsigned int)ch, (double)raw[ch], (double)or_v, (double)real_v);
        if (len > 0)
        {
            MLY_UART1_SEND("%s", line);
        }
    }
#endif
}

/* 任务入口 */
void ADS7953_Scan(void)
{
    ADS7953_ScanAll_Manual();
}
