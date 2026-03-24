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
#define ADS7953_CMD_ENABLE_PROG      0x0800U
#define ADS7953_CMD_RESET_CHAN       0x0400U
#define ADS7953_CMD_CH_SHIFT         7U
#define ADS7953_CMD_CH_MASK          0x0FU
/* RANGE: 0=0~VREF(2.5V), 1=0~2*VREF(5V) - 我们硬件用5V量程所以用1 */
#define ADS7953_CMD_RANGE_BIT        0x0040U
#define ADS7953_CMD_PWR_UP_BIT       0x0020U
//DEBUG INFO 可以0禁用 1打开 主要用于看SPI传输
#define ADS7953_DEBUG_RAW_FRAME      0U
#define ADS7953_DEBUG_RX_ALL_ZERO    1U

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
/* 前向声明：ADS7953_ReadWrite 必须在使用前声明 */
static uint8_t ADS7953_ReadWrite(uint16_t cmd16, uint16_t *rx16);

/* Manual模式命令字：
 * [15:12] = 0001 手动模式
 * [11]    = ENABLE_PROG（使能编程）
 * [10]    = RESET_CHAN（重置通道计数器，仅在模式切换时用）
 * [9:7]   = 通道号
 * [6]     = RANGE（0=0~VREF, 1=0~2*VREF）
 * [5]     = PWR_UP
 */
static inline uint16_t ADS7953_CmdManual(uint8_t ch)
{
    uint16_t cmd = ADS7953_CMD_MODE_MANUAL;
    cmd |= ADS7953_CMD_ENABLE_PROG;
    cmd |= (uint16_t)((uint16_t)(ch & ADS7953_CMD_CH_MASK) << ADS7953_CMD_CH_SHIFT);
    cmd |= ADS7953_CMD_RANGE_BIT;
    cmd |= ADS7953_CMD_PWR_UP_BIT;
    return cmd;
}

/* 带 RESET_CHAN 的命令，用于模式切换时重置通道计数器 */
static inline uint16_t ADS7953_CmdManualWithReset(void)
{
    uint16_t cmd = ADS7953_CMD_MODE_MANUAL;
    cmd |= ADS7953_CMD_ENABLE_PROG;
    cmd |= ADS7953_CMD_RESET_CHAN;
    cmd |= ADS7953_CMD_RANGE_BIT;
    cmd |= ADS7953_CMD_PWR_UP_BIT;
    return cmd;
}

/* 参考 TI 官方例程的 initialize_pipeline：
 * SPI pipeline 需要 2 帧来完成初始化
 * 第1帧：启动通道选择
 * 第2帧：开始采集，此时无有效数据返回
 */
static void ADS7953_InitPipeline(void)
{
    uint16_t cmd = ADS7953_CmdManualWithReset();
    uint16_t dummy;  /* 丢弃初始化期间的无用数据 */
    (void)ADS7953_ReadWrite(cmd, &dummy);  /* 第1帧：带RESET_CHAN，重置通道计数器 */
    OSIF_TimeDelay(1);
    (void)ADS7953_ReadWrite(cmd, &dummy);  /* 第2帧：开始采集，无有效数据 */
    OSIF_TimeDelay(1);
}



/* 参考 TI 官方例程，每次发送命令后：
 * response 包含的是"上一次命令对应的转换结果"
 * 即 response 的 tag = 上一次命令请求的通道
 */

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

/* 解析ADS7953返回帧：高4位tag、低12位采样值（按tag索引） */
static void ADS7953_DecodeAndStore(uint16_t rx)
{
    uint8_t tag = (uint8_t)((rx >> 12) & 0x0FU);
    uint16_t value = (uint16_t)(rx & 0x0FFFU);

    ADS7953_GetValueOrig[tag] = value;
    ADS7953_TagSeen[tag] = 1U;
    ADS7953_DataValid[tag] = 1U;
}

/* 按命令顺序索引存储值（忽略rx中的tag）
 * 由于ADS7953 pipeline延迟导致tag不可预测，改用发送顺序作为索引
 * ch: 发送的命令通道号
 * value: rx的低12位原始值
 */
static void ADS7953_StoreByChannel(uint8_t ch, uint16_t value)
{
    ADS7953_GetValueOrig[ch] = value;
    ADS7953_TagSeen[ch] = 1U;
    ADS7953_DataValid[ch] = 1U;
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
 * 参考 TI 官方 ads79xx.c 实现
 * 注意：由于ADS7953 pipeline延迟导致rx中的tag不可预测，
 *      所有通道数据改用发送顺序索引（ch）而非tag索引
 * 流程：
 *   1) ADS7953_InitPipeline() - 发送 RESET_CHAN + 2帧初始化
 *   2) 对每个通道发送2次命令：第1次触发转换，第2次读取结果
 *   3) 直接按ch索引存储rx的低12位值，忽略rx.tag
 *   4) 最后 flush 2帧确保 pipeline 清空
 */
void ADS7953_ScanAll_Manual(void)
{
    uint16_t rx = 0U;
    uint8_t missing = 0U;

    memset(ADS7953_TagSeen, 0, sizeof(ADS7953_TagSeen));
    memset(ADS7953_DataValid, 0, sizeof(ADS7953_DataValid));

    /* Step 1: 参考 TI initialize_pipeline - RESET_CHAN + 2帧初始化 */
    ADS7953_InitPipeline();

    /* Step 2: 对每个通道：
     * 第1次 ReadWrite: 触发转换，读取的是 pipeline 中残留的旧数据（丢弃）
     * 第2次 ReadWrite: 读取本次通道的转换结果
     * 直接用 ch 作为数组索引，忽略 rx 中的 tag */
    for (uint8_t ch = 0U; ch < ADS7953_CH_COUNT; ++ch)
    {
        /* 第1帧：触发 ch 的转换 */
        (void)ADS7953_ReadWrite(ADS7953_CmdManual(ch), &rx);
        OSIF_TimeDelay(1);

        /* 第2帧：读取 ch 的转换结果 */
        (void)ADS7953_ReadWrite(ADS7953_CmdManual(ch), &rx);
        OSIF_TimeDelay(1);

        if (ADS7953_IsRxFrameValid(rx) != 0U)
        {
            ADS7953_StoreByChannel(ch, (uint16_t)(rx & 0x0FFFU));
        }
    }

    /* Step 3: flush 2帧确保 pipeline 清空（用ch=0存储） */
    for (uint8_t flush = 0U; flush < 2U; ++flush)
    {
        (void)ADS7953_ReadWrite(ADS7953_CmdManual(0U), &rx);
        OSIF_TimeDelay(1);

        if (ADS7953_IsRxFrameValid(rx) != 0U)
        {
            ADS7953_StoreByChannel(0U, (uint16_t)(rx & 0x0FFFU));
        }
    }

    missing = 0U;
    for (uint8_t i = 0U; i < ADS7953_CH_COUNT; ++i)
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

    for (uint8_t i = 0U; i < ADS7953_CH_COUNT; ++i)
    {
        MLY_UART1_SEND("ch%u=%lu(v=%u) ", i, ADS7953_GetValueOrig[i], ADS7953_DataValid[i]);
    }

    MLY_UART1_SEND("| miss=%u validMap=0x%04X spiErr=%lu\r\n",
                   missing,
                   ADS7953_GetValidBitmap(),
                   ADS7953_SpiErrCnt);
#endif
}

/* 单通道扫描：只扫描指定通道，输出该通道值
 * 注意：改用发送顺序索引，忽略rx中的tag
 * 流程：
 * 1) ADS7953_InitPipeline() - 复位 + 初始化 pipeline
 * 2) 发送目标通道命令2次
 * 3) 用ch索引存储结果
 */
void ADS7953_Scan_channel(uint8_t ch)
{
    uint16_t rx = 0U;

    if (ch >= ADS7953_CH_COUNT)
    {
        MLY_UART1_SEND("ADS7953 ch invalid: %d\r\n", ch);
        return;
    }

    /* 清零所有通道状态 */
    memset(ADS7953_TagSeen, 0, sizeof(ADS7953_TagSeen));
    memset(ADS7953_DataValid, 0, sizeof(ADS7953_DataValid));

    /* Step 1: 复位 + 初始化 pipeline */
    ADS7953_InitPipeline();

    /* Step 2: 发送目标通道命令2次 */
    (void)ADS7953_ReadWrite(ADS7953_CmdManual(ch), &rx);  /* 第1帧：触发转换 */
    OSIF_TimeDelay(1);
    (void)ADS7953_ReadWrite(ADS7953_CmdManual(ch), &rx);  /* 第2帧：读取结果 */
    OSIF_TimeDelay(1);

    /* Step 3: 用ch索引存储结果，忽略rx.tag */
    if (ADS7953_IsRxFrameValid(rx) != 0U)
    {
        ADS7953_StoreByChannel(ch, (uint16_t)(rx & 0x0FFFU));
    }

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
