#include "USERINIT.h"
#include "LiuyunLPSPI2.h"
#include "osif.h"
uint32_t ADS7953_GetValueOrig[16] = {0};  

static inline uint16_t ADS7953_ReadWrite(uint16_t cmd16)
{
    uint8_t tx[2] = { cmd16 >> 8, cmd16 & 0xFF };
    uint8_t rx[2] = {0};
    LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, tx, rx, 2U,200u);
    return ((uint16_t)rx[0] << 8) | rx[1];
}

static inline uint16_t SPI1_PCS0_outputTest(uint16_t cmd16)
{
    uint8_t tx[2] = { cmd16 & 0xFF,cmd16 >> 8  };
    uint8_t rx[2] = {0};
    LPSPI_DRV_MasterTransferBlocking(LPSPICOM1, tx, rx, 2U,200u);
    return ((uint16_t)rx[0] << 8) | rx[1];
}

void ADS7953_ScanAll_Manual(void)
{
    static uint32_t prevRx = 0;
    for (uint8_t ADS7953_Manual_NextCh = 0; ADS7953_Manual_NextCh < 16; ++ADS7953_Manual_NextCh)
    {
        uint32_t cmd = 0x1C00 | (ADS7953_Manual_NextCh << 7) | 0x0040;
        uint32_t rx  = ADS7953_ReadWrite(cmd);  
        OSIF_TimeDelay(1);
 
        if (ADS7953_Manual_NextCh > 0) ADS7953_GetValueOrig[ADS7953_Manual_NextCh-1] = prevRx & 0x0FFF;
        prevRx = rx;
        ADS7953_GetValueOrig[ADS7953_Manual_NextCh] = rx & 0x0FFF; 
    }

    for (uint8_t i = 0; i < 16; ++i)
    {
        // MLY_UART1_SEND("ch=%d,val=%d ", i, ADS7953_GetValueOrig[i]);
        MLY_UART1_SEND("%d ", ADS7953_GetValueOrig[i]);
    }
    MLY_UART1_SEND("\r\n");
}
void ADS7953_ScanAll_Auto1Mode(void)
{
    static uint32_t prevRx = 0;
    uint32_t cmd = 0x3C40; 
    uint32_t rx = ADS7953_ReadWrite(cmd);
    OSIF_TimeDelay(1); 

    for (uint8_t ADS7953_Auto_NextCh = 0; ADS7953_Auto_NextCh < 16; ++ADS7953_Auto_NextCh)
    {
        rx = ADS7953_ReadWrite(0x3840); 
        OSIF_TimeDelay(1); 

        if (ADS7953_Auto_NextCh > 0) ADS7953_GetValueOrig[ADS7953_Auto_NextCh-1] = prevRx & 0x0FFF;
        prevRx = rx;
        ADS7953_GetValueOrig[ADS7953_Auto_NextCh] = rx & 0x0FFF;
    }
    for (uint8_t i = 0; i < 16; ++i)
    {
        MLY_UART1_SEND("%d ", ADS7953_GetValueOrig[i]);
    }
    MLY_UART1_SEND("\r\n");
}
void ADS7953_AUTO_2_TEST(void)
{
    uint16_t rx;
    uint8_t  data[16] = {0};          /* 12 个通道结果（每通道 1 Byte） */
    rx = ADS7953_ReadWrite(0x3C40);
    MLY_UART1_SEND("F0:%02X%02X\r\n", (rx>>8)&0xFF, rx&0x0FF);
    rx = ADS7953_ReadWrite(0x3840);
    MLY_UART1_SEND("F1:%02X%02X\r\n", (rx>>8)&0xFF, rx&0x0FF);
    for (uint8_t i = 0; i < 16; ++i)
    {
        rx = ADS7953_ReadWrite(0x3840);
        data[i] = (rx & 0x0FFF) >> 4;
        MLY_UART1_SEND("F%d:%02X%02X\r\n", i+2, (rx>>8)&0xFF, rx&0x0FF);
    }
    MLY_UART1_SEND("Auto2=");
    for (uint8_t i = 0; i < 16; ++i)
    {
        MLY_UART1_SEND("%02X", data[i]);
    }
    MLY_UART1_SEND("\r\n");
}
void ADS7953_Scan(void)
{
	uint16_t r5;
    r5 = ADS7953_ReadWrite(0x1A40) & 0x0FFF; //ADS7953接收到1AC0正确
    // r5 = SPI1_PCS0_outputTest(0x1AC0) & 0x0FFF;
    // r7 = ADS7953_ReadWrite(0x1BC0) & 0x0FFF;//ADS7953接收到0x1BC0正确
    MLY_UART1_SEND("%d\r\n",r5);
}

