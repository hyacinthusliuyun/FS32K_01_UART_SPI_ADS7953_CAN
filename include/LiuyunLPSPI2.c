#include "USERINIT.h"
#include "LiuyunLPSPI2.h"
#include "JustFloatUART.h"
#include "osif.h"

/*
    当前串口发送为请看JustFloatUART.C.H
    默认为justfloat协议，如不需要请更改上述.h的ADS7953_TX_USE_JUSTFLOAT为0
*/

#define ADS7953_CH_COUNT             16U
#define ADS7953_CMD_MODE_MANUAL      0x1000U
#define ADS7953_CMD_TAG_ENABLE       0x0800U
#define ADS7953_CMD_CH_SHIFT         7U
#define ADS7953_CMD_CH_MASK          0x0FU
#define ADS7953_CMD_RANGE_BIT        0x0040U
#define ADS7953_CMD_PWR_UP_BIT       0x0020U
//DEBUG INFO 可以0禁用 1打开 主要用于看SPI传输 
#define ADS7953_DEBUG_RAW_FRAME      0U
#define ADS7953_DEBUG_RX_ALL_ZERO    0U

/* 16通道原始采样缓存（按返回tag索引） */
uint32_t ADS7953_GetValueOrig[ADS7953_CH_COUNT] = {0};
/* 1=本轮扫描该通道收到过有效SPI回包并完成更新；0=未更新 */
uint8_t ADS7953_DataValid[ADS7953_CH_COUNT] = {0};

uint32_t ADS7953_SpiErrCnt = 0;
uint32_t ADS7953_SpiOkCnt = 0;
/* 统计本轮"缺失通道"次数（按tag未覆盖） */
uint32_t ADS7953_TagErrCnt = 0;

/* 本轮扫描中各通道是否收到过有效tag */
static uint8_t ADS7953_TagSeen[ADS7953_CH_COUNT] = {0};
/* 最近一次SPI传输状态 */
static status_t ADS7953_LastSpiStatus = STATUS_SUCCESS;

/* Manual模式命令字：
 * [15:12] = 0001 手动模式
 * [11]    = TAG使能（返回高4位用于通道标识）
 * [10:7]  = 通道号
 * [6]     = 保持原工程配置位（量程/配置相关）
 */
static inline uint16_t ADS7953_CmdManual(uint8_t ch)
{
    uint16_t cmd = ADS7953_CMD_MODE_MANUAL;
    cmd |= ADS7953_CMD_TAG_ENABLE;
    cmd |= (uint16_t)((uint16_t)(ch & ADS7953_CMD_CH_MASK) << ADS7953_CMD_CH_SHIFT);
    cmd |= ADS7953_CMD_RANGE_BIT;
    cmd |= ADS7953_CMD_PWR_UP_BIT;
    return cmd;
}

/* 发送1帧16bit命令并读取1帧16bit返回；成功返回1，失败返回0 */
static uint8_t ADS7953_ReadWrite(uint16_t cmd16, uint16_t *rx16)
{
    uint8_t tx[2] = {(uint8_t)(cmd16 >> 8), (uint8_t)(cmd16 & 0xFFU)};
    uint8_t rx[2] = {0};

    ADS7953_LastSpiStatus = LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    if (ADS7953_LastSpiStatus != STATUS_SUCCESS)
    {
        ADS7953_SpiErrCnt++;
        return 0U;
    }

    ADS7953_SpiOkCnt++;
    *rx16 = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);

#if ADS7953_DEBUG_RAW_FRAME
    MLY_UART1_SEND("ADS7953_RAW cmd=0x%04X rx=0x%04X st=%d\r\n",
                   cmd16,
                   *rx16,
                   (int)ADS7953_LastSpiStatus);
#endif

    return 1U;
}

static inline uint8_t ADS7953_IsRxFrameValid(uint16_t rx)
{
#if ADS7953_DEBUG_RX_ALL_ZERO
    return (rx != 0x0000U) ? 1U : 0U;
#else
    (void)rx;
    return 1U;
#endif
}

/* 解析ADS7953返回帧：高4位tag、低12位采样值 */
static void ADS7953_DecodeAndStore(uint16_t rx)
{
    uint8_t tag = (uint8_t)((rx >> 12) & 0x0FU);
    uint16_t value = (uint16_t)(rx & 0x0FFFU);

    ADS7953_GetValueOrig[tag] = value;
    ADS7953_TagSeen[tag] = 1U;
    ADS7953_DataValid[tag] = 1U;
}

uint16_t ADS7953_GetValidBitmap(void)
{
    uint16_t bitmap = 0U;

    for (uint8_t i = 0U; i < ADS7953_CH_COUNT; ++i)
    {
        if (ADS7953_DataValid[i] != 0U)
        {
            bitmap |= (uint16_t)(1U << i);
        }
    }

    return bitmap;
}

/* Manual模式整帧扫描：
 * 1) 连续下发16个通道命令
 * 2) 追加2帧flush，尽量覆盖pipeline延迟
 * 3) 输出16通道值与缺失通道计数
 */
void ADS7953_ScanAll_Manual(void)
{
    uint16_t rx = 0U;
    uint8_t missing = 0U;

    memset(ADS7953_TagSeen, 0, sizeof(ADS7953_TagSeen));
    memset(ADS7953_DataValid, 0, sizeof(ADS7953_DataValid));

    for (uint8_t ch = 0; ch < ADS7953_CH_COUNT; ++ch)
    {
        if (ADS7953_ReadWrite(ADS7953_CmdManual(ch), &rx) != 0U)
        {
            if (ADS7953_IsRxFrameValid(rx) != 0U)
            {
                ADS7953_DecodeAndStore(rx);
            }
        }
        OSIF_TimeDelay(1);
    }

    for (uint8_t flush = 0; flush < 2U; ++flush)
    {
        if (ADS7953_ReadWrite(ADS7953_CmdManual(0U), &rx) != 0U)
        {
            if (ADS7953_IsRxFrameValid(rx) != 0U)
            {
                ADS7953_DecodeAndStore(rx);
            }
        }
        OSIF_TimeDelay(1);
    }

    for (uint8_t i = 0; i < ADS7953_CH_COUNT; ++i)
    {
        if (ADS7953_TagSeen[i] == 0U)
        {
            missing++;
        }
    }

    ADS7953_TagErrCnt += (uint32_t)missing;

#if ADS7953_TX_USE_JUSTFLOAT
    JFLOAT_UART_SendAds16RawAsFloat(ADS7953_GetValueOrig, ADS7953_DataValid);
#else
    if (missing == ADS7953_CH_COUNT)
    {
        MLY_UART1_SEND("ADS7953_NO_VALID_SAMPLE ");
    }

    for (uint8_t i = 0; i < ADS7953_CH_COUNT; ++i)
    {
        MLY_UART1_SEND("ch%u=%lu(v=%u) ", i, ADS7953_GetValueOrig[i], ADS7953_DataValid[i]);
    }

    MLY_UART1_SEND("| miss=%u validMap=0x%04X spiErr=%lu\r\n",
                   missing,
                   ADS7953_GetValidBitmap(),
                   ADS7953_SpiErrCnt);
#endif
}

/* 单通道扫描：只扫描指定通道，输出该通道值 */
void ADS7953_Scan_channel(uint8_t ch)
{
    uint16_t rx = 0U;

    if (ch >= ADS7953_CH_COUNT)
    {
        MLY_UART1_SEND("ADS7953 ch invalid: %d\r\n", ch);
        return;
    }

    /* 清零该通道状态 */
    ADS7953_TagSeen[ch] = 0U;
    ADS7953_DataValid[ch] = 0U;

    /* 发送单通道命令并读取 */
    (void)ADS7953_ReadWrite(ADS7953_CmdManual(ch), &rx);

    /* 追加1帧flush */
    OSIF_TimeDelay(1);
    (void)ADS7953_ReadWrite(ADS7953_CmdManual(ch), &rx);

#if ADS7953_TX_USE_JUSTFLOAT
    JFLOAT_UART_SendAds16RawAsFloat(ADS7953_GetValueOrig, ADS7953_DataValid);
#else
    MLY_UART1_SEND("ch%u=%lu(v=%u) spiErr=%lu\r\n",
                   ch,
                   ADS7953_GetValueOrig[ch],
                   ADS7953_DataValid[ch],
                   ADS7953_SpiErrCnt);
#endif
}

/* 任务入口：当前固定走单通道扫描（CH13） */
void ADS7953_Scan(void)
{
    ADS7953_ScanAll_Manual();
    // ADS7953_Scan_channel(8U);
}
