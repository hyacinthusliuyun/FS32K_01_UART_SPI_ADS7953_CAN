#include "USERINIT.h"
#include "LiuyunLPSPI2.h"
#include "osif.h"

/* 16通道原始采样缓存（按返回tag索引） */
uint32_t ADS7953_GetValueOrig[16] = {0};

uint32_t ADS7953_SpiErrCnt = 0;
uint32_t ADS7953_SpiOkCnt = 0;
/* 返回tag异常计数 */
uint32_t ADS7953_TagErrCnt = 0;

/* 本轮扫描中各通道是否收到过有效tag */
static uint8_t ADS7953_TagSeen[16] = {0};
/* 最近一次SPI传输状态 */
static status_t ADS7953_LastSpiStatus = STATUS_SUCCESS;

/* Manual模式命令字：
 * [15:12]=0001 Manual
 * [10:7] = 通道号
 * [6]    = 1（当前实现沿用原工程配置位）
 */
static inline uint16_t ADS7953_CmdManual(uint8_t ch)
{
    return (uint16_t)(0x1C00U | ((uint16_t)(ch & 0x0FU) << 7) | 0x0040U);
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
    return 1U;
}

/* 解析ADS7953返回帧：高4位tag、低12位采样值 */
static void ADS7953_DecodeAndStore(uint16_t rx)
{
    //tag校验ch对应value
    uint8_t tag = (uint8_t)((rx >> 12) & 0x0FU);
    uint16_t value = (uint16_t)(rx & 0x0FFFU);

    if (tag < 16U)
    {
        ADS7953_GetValueOrig[tag] = value;
        ADS7953_TagSeen[tag] = 1U;
    }
    else
    {
        ADS7953_TagErrCnt++;
    }
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

    for (uint8_t ch = 0; ch < 16U; ++ch)
    {
        if (ADS7953_ReadWrite(ADS7953_CmdManual(ch), &rx) != 0U)
        {
            ADS7953_DecodeAndStore(rx);
        }
        OSIF_TimeDelay(1);
    }

    for (uint8_t flush = 0; flush < 2U; ++flush)
    {
        if (ADS7953_ReadWrite(ADS7953_CmdManual(0U), &rx) != 0U)
        {
            ADS7953_DecodeAndStore(rx);
        }
        OSIF_TimeDelay(1);
    }

    for (uint8_t i = 0; i < 16U; ++i)
    {
        if (ADS7953_TagSeen[i] == 0U)
        {
            missing++;
        }
        MLY_UART1_SEND("%d ", ADS7953_GetValueOrig[i]);
    }
    MLY_UART1_SEND("| miss=%d spiErr=%lu\r\n", missing, ADS7953_SpiErrCnt);
}

/* 单通道快速检查：用于调试时快速确认某一通道缓存是否更新 */
void ADS7953_CheckChannel(uint8_t ch)
{
    if (ch >= 16U)
    {
        MLY_UART1_SEND("ADS7953 ch invalid: %d\r\n", ch);
        return;
    }

    MLY_UART1_SEND("ADS7953_CH%d=%d tagSeen=%d spiErr=%lu\r\n",
                   ch,
                   ADS7953_GetValueOrig[ch],
                   ADS7953_TagSeen[ch],
                   ADS7953_SpiErrCnt);
}

/* 任务入口：当前固定走Manual整帧扫描 */
void ADS7953_Scan(void)
{
    ADS7953_ScanAll_Manual();
}
