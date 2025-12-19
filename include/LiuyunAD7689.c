#include "LiuyunAD7689.h"

LiuyunAD7689_t AD7689 = {0};

/* 私有：16bit SPI 传输（高字节先发）*/
static uint16_t AD7689_Transfer16(uint16_t txData)
{
    uint8_t tx[2] = { txData >> 8, txData & 0xFF };
    uint8_t rx[2] = {0};

    LPSPI_DRV_MasterTransferBlocking(AD7689_LPSPI_INSTANCE, tx, rx, 2U, 500U);
    return ((uint16_t)rx[0] << 8) | rx[1];
}

/* 初始化 */
void AD7689_Init(void)
{
    // CNV 上电默认高
    AD7689_CNV_H();
    OSIF_TimeDelay(1);

    // 上电后丢弃前 8 次无效转换
    for (int i = 0; i < 8; i++)
    {
        AD7689_ReadChannel(0);
        OSIF_TimeDelay(2);
    }
}

/* 单通道读取（核心函数，绝对稳）*/
uint16_t AD7689_ReadChannel(uint8_t ch)
{
    if (ch > 7) ch = 0;

    /* 构造配置字 - 参考 AD7689 数据手册 Table 24
       [13]   = 1     : 更新配置寄存器
       [12:10]= 111   : 全带宽, 外部 REF, 使能 REF
       [9:7]  = 000   : 单端输入，以 COM 为参考
       [6:3]  = ch    : 选择通道 IN0~IN7
       [2:1]  = 00     : 禁用序列器
       [0]    = 1     : 读回配置寄存器
    */
    uint16_t cfg = 0x8000                  // bit13=1 写配置
                 | (0x7 << 10)              // 全带宽 + 外部 REF + 使能 REF
                 | (0x0 << 7)               // 单端输入
                 | (ch << 3)                // 通道选择
                 | 0x1;                     // bit0=1 读回配置

    /* 第1步：拉低 CNV 启动转换（t_CONV = 1.6us）*/
    AD7689_CNV_L();
    OSIF_TimeDelay(2);                      // 至少 2us

    /* 第2步：拉高 CNV + 等待建立时间（关键！）*/
    AD7689_CNV_H();
    OSIF_TimeDelay(AD7689_ACQ_DELAY_US);    // 10us 以上绝对稳

    /* 第3步：发送配置字，收到上一帧垃圾数据 */
    uint16_t dummy = AD7689_Transfer16(cfg);

    /* 第4步：再次发送任意数据，读取本次真实结果 */
    uint16_t result = AD7689_Transfer16(0x0000);

    /* 解析 */
    uint16_t raw    = result & 0xFFFF;
    uint8_t  chBack = (result >> 13) & 0x07;

    AD7689.ch           = chBack;
    AD7689.raw[chBack]  = raw;
    AD7689.volt[chBack] = raw * AD7689_VREF / 65536.0f;
    AD7689.cfgReadback  = dummy >> 2;       // 调试用

    /* 完全复刻你原来的应用层换算逻辑 */
    if (chBack < 4) {
        // 差分通道：放大 51 倍 → uV
        AD7689.vin[chBack] = (AD7689.volt[chBack] / 51.0f) * 2000000.0f;
    } else {
        // 单端外部输入
        AD7689.vin[chBack] = 4.0f * AD7689.volt[chBack] - 5.0f;
    }

    return raw;
}

/* 扫描所有通道（推荐使用）*/
void AD7689_ScanAllChannels(void)
{
    for (uint8_t ch = 0; ch < 8; ch++)
    {
        AD7689_ReadChannel(ch);
        // 通道间额外延时（可选，越稳越好）
        OSIF_TimeDelay(2);
    }
}