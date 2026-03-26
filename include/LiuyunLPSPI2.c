#include "USERINIT.h"
#include "LiuyunLPSPI2.h"
#include "JustFloatUART.h"
#include "osif.h"
/*默认为justfloat协议ADS7953_TX_USE_JUSTFLOAT*/

#define ADS7953_CH_COUNT             16U
#define ADS7953_CMD_MODE_MANUAL      0x1000U
#define ADS7953_CMD_ENABLE_PROG      0x0800U
#define ADS7953_CMD_RESET_CHAN       0x0400U
#define ADS7953_CMD_CH_SHIFT         7U
#define ADS7953_CMD_CH_MASK          0x0FU
/* RANGE: 0=0~VREF(2.5V), 1=0~2*VREF(5V) - 我们硬件用5V量程所以用1 */
#define ADS7953_CMD_RANGE_BIT        0x0040U
#define ADS7953_CMD_PWR_UP_BIT       0x0020U
#define ADS7953_DEBUG_RAW_FRAME      0U
#define ADS7953_DEBUG_RX_ALL_ZERO    1U

/* 16通道原始采样缓存（按返回tag索引） */
uint32_t ADS7953_GetValueOrig[ADS7953_CH_COUNT] = {0};
/* 1=本轮扫描该通道收到过有效SPI回包并完成更新；0=未更新 */
uint8_t ADS7953_DataValid[ADS7953_CH_COUNT] = {0};

uint32_t ADS7953_SpiErrCnt = 0;
uint32_t ADS7953_SpiOkCnt = 0;
/* 本轮缺失通道次数 */
uint32_t ADS7953_TagErrCnt = 0;
/* 有效tag */
static uint8_t ADS7953_TagSeen[ADS7953_CH_COUNT] = {0};
/* 最近一次SPI传输状态 */
static status_t ADS7953_LastSpiStatus = STATUS_SUCCESS;

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

/* 参考 TI 官方 ads79xx.c 的 initialize_pipeline：
 * 不使用 RESET_CHAN，只发 2 帧相同命令
 * 第1帧：启动通道选择
 * 第2帧：开始采集，此时无有效数据返回（2帧结果都丢弃）
 */
static void ADS7953_InitPipeline(void)
{
    uint16_t cmd = ADS7953_CmdManual(0U);  /* 不带RESET_CHAN，使用通道0 */
    uint16_t dummy;
    (void)ADS7953_ReadWrite(cmd, &dummy);  /* 第1帧 */
    (void)ADS7953_ReadWrite(cmd, &dummy);  /* 第2帧 */
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
 * 参考 TI 官方 ads79xx.c 的 CaptureConversions 模式
 *
 * ADS7953 pipeline 核心原则：每次发送命令后，response 包含的是上一次转换的结果
 *
 * 正确时序（共 18+1=19 帧，得到 16 个有效结果）：
 *   帧0-1: ADS7953_InitPipeline() - 2帧初始化，结果丢弃
 *   帧2:   发送 cmd(0), 接收 = init最后1帧的结果（垃圾）
 *   帧3:   发送 cmd(1), 接收 = ch0 的结果  → 存入 ch0
 *   帧4:   发送 cmd(2), 接收 = ch1 的结果  → 存入 ch1
 *   ...
 *   帧17:  发送 cmd(15),接收 = ch14的结果  → 存入 ch14
 *   帧18:  发送 cmd(0), 接收 = ch15的结果  → 存入 ch15
 *
 * 总共 19 帧（2 init + 16 scan + 1 flush），得到 ch0~ch15 共 16 个有效结果
 */
void ADS7953_ScanAll_Manual(void)
{
    uint16_t rx = 0U;
    uint16_t cmd = 0U;
    uint8_t missing = 0U;

    memset(ADS7953_TagSeen, 0, sizeof(ADS7953_TagSeen));
    memset(ADS7953_DataValid, 0, sizeof(ADS7953_DataValid));

    /* Step 1: TI initialize_pipeline - 2帧初始化，结果丢弃 */
    ADS7953_InitPipeline();

    /* Step 2: 顺序发送 cmd(0)~cmd(15)，每次接收上一帧的结果
     * rx[ch_idx] 包含的是上一次命令的结果
     * ch_idx=0 时 rx 是 init 的垃圾数据，丢弃
     * ch_idx=1 时 rx 是 ch0 的结果，存入 ch0
     * ...
     * ch_idx=15 时 rx 是 ch14 的结果，存入 ch14
     */
    for (uint8_t ch_idx = 0U; ch_idx < ADS7953_CH_COUNT; ++ch_idx)
    {
        cmd = ADS7953_CmdManual(ch_idx);
        (void)ADS7953_ReadWrite(cmd, &rx);

        /* ch_idx=0 时 rx 是垃圾，ch_idx>=1 时 rx 才是对应通道的结果 */
        if ((ch_idx >= 1U) && (ADS7953_IsRxFrameValid(rx) != 0U))
        {
            ADS7953_StoreByChannel(ch_idx - 1U, (uint16_t)(rx & 0x0FFFU));
        }
    }

    /* Step 3: 再发 1 帧 cmd(0)，把 ch15 的结果读出来 */
    (void)ADS7953_ReadWrite(ADS7953_CmdManual(0U), &rx);
    if (ADS7953_IsRxFrameValid(rx) != 0U)
    {
        ADS7953_StoreByChannel(15U, (uint16_t)(rx & 0x0FFFU));
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
 *
 * ADS7953 pipeline 原则：response 包含上一次命令的结果
 *
 * 正确流程（共 4 帧）：
 *   帧0-1: ADS7953_InitPipeline() - 2帧初始化，结果丢弃
 *   帧2:   发送 cmd(ch), 接收 = init最后1帧的结果（垃圾，丢弃）
 *   帧3:   发送 cmd(ch), 接收 = ch 的结果（有效，存入 ch）
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

    /* Step 1: TI initialize_pipeline - 2帧初始化 */
    ADS7953_InitPipeline();

    /* Step 2: 第1次发送 cmd(ch)，接收垃圾（init 最后1帧的结果，丢弃） */
    (void)ADS7953_ReadWrite(ADS7953_CmdManual(ch), &rx);

    /* Step 3: 第2次发送 cmd(ch)，接收 ch 的转换结果（有效） */
    (void)ADS7953_ReadWrite(ADS7953_CmdManual(ch), &rx);
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

/* 单帧测试：每帧命令后等待片刻，看 ADS7953 是否能进入 Manual Mode
 *
 * 测试思路：
 * 1. 先发 RESET 命令让 ADS7953 进入已知状态
 * 2. 再发 MANUAL 命令，看 tag 是否变为 1
 *
 * 只发一帧命令，中间隔一段时间，确保 ADS7953 有时间处理
 */
void ADS7953_TestSingleFrame(void)
{
    uint16_t cmd;
    uint8_t tx[2];
    uint8_t rx[2] = {0};
    volatile uint32_t delay;

    /* 测试0: 发送 RESET 命令 (0x1C60)，让 ADS7953 进入 Manual 模式
     * 根据 datasheet，RESET_CHAN 位可重置通道计数器，使能编程 */
    cmd = 0x1C60U;
    tx[0] = (uint8_t)(cmd >> 8);
    tx[1] = (uint8_t)(cmd & 0xFFU);
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);

    /* 延时一段时间，让 ADS7953 处理 */
    for (delay = 0; delay < 10000U; delay++) { }

    /* 再发一帧同样的命令，看 tag 是否为 1 */
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    MLY_UART1_SEND("T0 afterReset rx=%02X%02X tag=%u\r\n",
                   rx[0], rx[1], (unsigned)((rx[0] >> 4) & 0x0FU));

    /* 延时 */
    for (delay = 0; delay < 10000U; delay++) { }

    /* 测试1: 0x1860 - 标准 Manual 命令 (无 RESET) */
    cmd = 0x1860U;
    tx[0] = (uint8_t)(cmd >> 8);
    tx[1] = (uint8_t)(cmd & 0xFFU);
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    for (delay = 0; delay < 10000U; delay++) { }
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    MLY_UART1_SEND("T1 cmd=1860 rx=%02X%02X tag=%u\r\n",
                   rx[0], rx[1], (unsigned)((rx[0] >> 4) & 0x0FU));

    /* 延时 */
    for (delay = 0; delay < 10000U; delay++) { }

    /* 测试2: 0x1C60 - 带 RESET_CHAN */
    cmd = 0x1C60U;
    tx[0] = (uint8_t)(cmd >> 8);
    tx[1] = (uint8_t)(cmd & 0xFFU);
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    for (delay = 0; delay < 10000U; delay++) { }
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    MLY_UART1_SEND("T2 cmd=1C60 rx=%02X%02X tag=%u\r\n",
                   rx[0], rx[1], (unsigned)((rx[0] >> 4) & 0x0FU));

    /* 延时 */
    for (delay = 0; delay < 10000U; delay++) { }

    /* 测试3: 0x1800 - 无 ENABLE_PROG (手册说此位必须为1才能编程) */
    cmd = 0x1800U;
    tx[0] = (uint8_t)(cmd >> 8);
    tx[1] = (uint8_t)(cmd & 0xFFU);
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    for (delay = 0; delay < 10000U; delay++) { }
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    MLY_UART1_SEND("T3 cmd=1800 rx=%02X%02X tag=%u\r\n",
                   rx[0], rx[1], (unsigned)((rx[0] >> 4) & 0x0FU));

    /* 延时 */
    for (delay = 0; delay < 10000U; delay++) { }

    /* 测试4: 0x9D60 - 最高位为1的命令 */
    cmd = 0x9D60U;
    tx[0] = (uint8_t)(cmd >> 8);
    tx[1] = (uint8_t)(cmd & 0xFFU);
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    for (delay = 0; delay < 10000U; delay++) { }
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    MLY_UART1_SEND("T4 cmd=9D60 rx=%02X%02X tag=%u\r\n",
                   rx[0], rx[1], (unsigned)((rx[0] >> 4) & 0x0FU));

    /* 延时 */
    for (delay = 0; delay < 10000U; delay++) { }

    /* 测试5: 0xA000 - Auto-2 模式 (PWR_UP=0)
     * [15:12]=1010=Auto2, [11]=0=ENABLE_PROG关闭, [5]=0=PWR_UP关闭 */
    cmd = 0xA000U;
    tx[0] = (uint8_t)(cmd >> 8);
    tx[1] = (uint8_t)(cmd & 0xFFU);
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    for (delay = 0; delay < 10000U; delay++) { }
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    MLY_UART1_SEND("T5 cmd=A000 rx=%02X%02X tag=%u\r\n",
                   rx[0], rx[1], (unsigned)((rx[0] >> 4) & 0x0FU));

    /* 延时 */
    for (delay = 0; delay < 10000U; delay++) { }

    /* 测试6: 0xE000 - Auto-2 模式 (PWR_UP=1)
     * [15:12]=1110=Auto2?, [11]=0=ENABLE_PROG关闭, [5]=1=PWR_UP开启 */
    cmd = 0xE000U;
    tx[0] = (uint8_t)(cmd >> 8);
    tx[1] = (uint8_t)(cmd & 0xFFU);
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    for (delay = 0; delay < 10000U; delay++) { }
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    MLY_UART1_SEND("T6 cmd=E000 rx=%02X%02X tag=%u\r\n",
                   rx[0], rx[1], (unsigned)((rx[0] >> 4) & 0x0FU));

    /* 延时 */
    for (delay = 0; delay < 10000U; delay++) { }

    /* 测试7: 0x8000 - Auto-1 模式
     * [15:12]=1000=Auto1, [11]=0, [5]=0 */
    cmd = 0x8000U;
    tx[0] = (uint8_t)(cmd >> 8);
    tx[1] = (uint8_t)(cmd & 0xFFU);
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    for (delay = 0; delay < 10000U; delay++) { }
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    MLY_UART1_SEND("T7 cmd=8000 rx=%02X%02X tag=%u\r\n",
                   rx[0], rx[1], (unsigned)((rx[0] >> 4) & 0x0FU));

    /* 延时 */
    for (delay = 0; delay < 10000U; delay++) { }

    /* 测试8: 0xC000 - Auto-2 变体 (PWR_UP=0) */
    cmd = 0xC000U;
    tx[0] = (uint8_t)(cmd >> 8);
    tx[1] = (uint8_t)(cmd & 0xFFU);
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    for (delay = 0; delay < 10000U; delay++) { }
    (void)LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U, 200U);
    MLY_UART1_SEND("T8 cmd=C000 rx=%02X%02X tag=%u\r\n",
                   rx[0], rx[1], (unsigned)((rx[0] >> 4) & 0x0FU));
}

/* 任务入口：当前固定走单通道扫描（CH13） */
void ADS7953_Scan(void)
{
    ADS7953_ScanAll_Manual();
    // ADS7953_Scan_channel(8U);
}
