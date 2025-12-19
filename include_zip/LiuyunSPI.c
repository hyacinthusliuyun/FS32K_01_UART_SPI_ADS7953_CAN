/* LiuyunSPI.c  —— 手动轮扫 16 通道，TI ADS7953 Manual Mode */
//GB2312----GB2312----GB2312----GB2312----GB2312----GB2312----GB2312----GB2312----GB2312----GB2312----
#include "LiuyunSPI.h"
#include "spi_pal.h"
#include "osif.h"
#define PCS_ADS7953    1u //PTE10      
uint32_t ADS7953_GetValueOrig[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};   // 12-bit 原始结果

 /* 2. 送出 Maunal 命令字
     *    DI[15:12]=0x1  Maunal                 0001
     *    DI[11]   =1   使能 DI[10:0]           0
     *    DI[10]   =0   从 CH0 开始             0000
     *    DI[9:7]  =0   任意 0000->CHANNEL 1 0001->CHANNEL2
     *    DI[6]    =1   2×VREF 量程             0
     *    DI[5]    =1   非低功耗                0
     *    DI[4]    =0   DO[15:12] 输出通道号    0
     *    DI[3:0]  =0   GPIO 无关               0000
     *    
     *   0000 0000 0000 0000
     *   0001 0 0000 0 0 0 0000
     *   0001 1 0000 1 0 0 0000
     *   0001 1000 0100 0000
     *   纯纯手动，不用自动模式了
     *   0X1840
     */


static inline uint32_t ADS7953_ReadWrite(uint32_t tx)
{
    uint32_t rx;
    /* 16-bit1帧*/
    SPI_MasterTransfer(&spi2Instance,&tx,&rx,1U);
//    SPI_MasterTransferBlocking(&spi2Instance,&tx,&rx,1U,200U);
    return rx;
}

void ADS7953_ScanAll_Manual(uint8_t pcsIdx)
{
    static uint32_t prevRx = 0;
    if (pcsIdx != 1) return;

    for (uint8_t ADS7953_Manual_NextCh = 0; ADS7953_Manual_NextCh < 16; ++ADS7953_Manual_NextCh)
    {
        /* Manual :          0x1 + ADS7953_Manual_NextCh[10:7] + 2×VREF, 非低功耗 */
        uint32_t cmd = 0x1C00 | (ADS7953_Manual_NextCh << 7) | 0x0040;

        PINS_DRV_WritePin(PTE, 10, 0);
        uint32_t rx  = ADS7953_ReadWrite(cmd);  
        PINS_DRV_WritePin(PTE, 10, 1);
        OSIF_TimeDelay(1);
 
        if (ADS7953_Manual_NextCh > 0) ADS7953_GetValueOrig[ADS7953_Manual_NextCh-1] = prevRx & 0x0FFF;
        prevRx = rx;
        ADS7953_GetValueOrig[ADS7953_Manual_NextCh] = rx & 0x0FFF;       /* 12 位结果 */
    }
    // 将所有的通道值输出在同一行
    for (uint8_t i = 0; i < 16; ++i)
    {
        // MLY_UART1_SEND("ch=%d,val=%d ", i, ADS7953_GetValueOrig[i]);
        MLY_UART1_SEND("%d ", ADS7953_GetValueOrig[i]);
    }
    MLY_UART1_SEND("\r\n");
}

/* ADS7953_ScanAll_Auto1Mode - 自动模式扫描所有通道 */
void ADS7953_ScanAll_Auto1Mode(uint8_t pcsIdx)
{
    static uint32_t prevRx = 0;
    if (pcsIdx != 1) return;

 
    uint32_t cmd = 0x3840; // 自动模式，从CH0开始，2×VREF量程，非低功耗

    PINS_DRV_WritePin(PTE, 10, 0); 
    uint32_t rx = ADS7953_ReadWrite(cmd);
    PINS_DRV_WritePin(PTE, 10, 1); 
    OSIF_TimeDelay(1); 

    // 读取所有通道的结果
    for (uint8_t ADS7953_Auto_NextCh = 0; ADS7953_Auto_NextCh < 16; ++ADS7953_Auto_NextCh)
    {
        PINS_DRV_WritePin(PTE, 10, 0); // 选中ADS7953
        rx = ADS7953_ReadWrite(0x3840); // 读取数据
        PINS_DRV_WritePin(PTE, 10, 1); // 取消选中ADS7953
        OSIF_TimeDelay(1); // 延时

        if (ADS7953_Auto_NextCh > 0) ADS7953_GetValueOrig[ADS7953_Auto_NextCh-1] = prevRx & 0x0FFF;
        prevRx = rx;
        ADS7953_GetValueOrig[ADS7953_Auto_NextCh] = rx & 0x0FFF; // 存储
    }
    for (uint8_t i = 0; i < 16; ++i)
    {
        // MLY_UART1_SEND("ch=%d,val=%d ", i, ADS7953_GetValueOrig[i]);
        MLY_UART1_SEND("%d ", ADS7953_GetValueOrig[i]);
    }
    MLY_UART1_SEND("\r\n");
}



void ADS7953_Scan(void)
{
    // ADS7953_ScanAll_Manual(PCS_ADS7953);
    // MLY_UART1_SEND("MLY_DEBUG_spi2 ADS7953 Manual\r\n");
    ADS7953_ScanAll_Auto1Mode(PCS_ADS7953);
    // MLY_UART1_SEND("MLY_DEBUG_spi2 ADS7953 Auto1Mode\r\n");
}
