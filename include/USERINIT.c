    #include "USERINIT.h"
    //utf-8
  
    uint8_t RX0BUFFER[256];
    uint16_t task_timer;
    uint8_t txData[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
    uint8_t DEBUG_MSG[128] = {0};
    uint8_t ADS7953_rst_TX[2] = {0x00,0x40};
    uint8_t ADS7953_rst_RX[2] = {0};

    uint8_t ADS7953_RX5[2];
    uint8_t ADS7953_RX7[2];

void USERINIT(void)
{
    /* SPI初始化与复位发送的返回状态 */
    status_t spiStatus;

    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
                   g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

    // GPIO
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    // LED
    LED1_ON();
    LED2_OFF();
    LED1_FLAG=0;

    //IQR
    INT_SYS_EnableIRQGlobal();

    //UART
    UART_Init(&uart_pal1_instance, &uart_pal1_Config0);
    MLY_UART1_SEND("INFO:UART TX\r\n");
    UART_ReceiveData(&uart_pal1_instance, RX0BUFFER, 256);
    // MLY_UART1_SEND("INFO:ADC INIT TX OK\r\n");

    //CAN
    APP_CAN_Init();
    MLY_UART1_SEND("INFO:CAN INIT\r\n");
    APP_CAN_Send(0x123, txData, 8);   //CAN_SEND

    //SPI
    /* LPSPI1初始化与PCS配置，失败时打印错误码，避免静默失败 */
    spiStatus = LPSPI_DRV_MasterInit(LPSPICOM1, &lpspiCom1State, &lpspiCom1_MasterConfig0);
    if (spiStatus != STATUS_SUCCESS)
    {
        MLY_UART1_SEND("ERR:LPSPI1 init fail:%d\r\n", (int)spiStatus);
    }

    spiStatus = LPSPI_DRV_SetPcs(LPSPICOM1, LPSPI_PCS0, LPSPI_ACTIVE_LOW);
    /* 设置LPSPI1片选极性与通道 */
    if (spiStatus != STATUS_SUCCESS)
    {
        MLY_UART1_SEND("ERR:LPSPI1 setpcs fail:%d\r\n", (int)spiStatus);
    }

    spiStatus = LPSPI_DRV_MasterTransferBlocking(LPSPICOM1, ADS7953_rst_TX, ADS7953_rst_RX, 2U, TIMEOUT);
    /* 发送一次16bit复位/同步帧，拉齐设备上电后的首帧状态 */
    if (spiStatus != STATUS_SUCCESS)
    {
        MLY_UART1_SEND("ERR:LPSPI1 rst tx fail:%d\r\n", (int)spiStatus);
    }

    spiStatus = LPSPI_DRV_MasterInit(LPSPICOM2, &lpspiCom2State, &lpspiCom2_MasterConfig0);
    /* LPSPI2为ADS7953主采集链路：初始化失败需立即可见 */
    if (spiStatus != STATUS_SUCCESS)
    {
        MLY_UART1_SEND("ERR:LPSPI2 init fail:%d\r\n", (int)spiStatus);
    }

    spiStatus = LPSPI_DRV_SetPcs(LPSPICOM2, LPSPI_PCS1, LPSPI_ACTIVE_LOW);
    /* LPSPI2使用PCS1，对应当前ADS7953片选连线 */
    if (spiStatus != STATUS_SUCCESS)
    {
        MLY_UART1_SEND("ERR:LPSPI2 setpcs fail:%d\r\n", (int)spiStatus);
    }

    spiStatus = LPSPI_DRV_MasterTransferBlocking(LPSPICOM2, ADS7953_rst_TX, ADS7953_rst_RX, 2U, TIMEOUT);
    /* ADS7953链路启动帧，便于后续Manual扫描做帧对齐 */
    if (spiStatus != STATUS_SUCCESS)
    {
        MLY_UART1_SEND("ERR:LPSPI2 rst tx fail:%d\r\n", (int)spiStatus);
    }

    MLY_UART1_SEND("INFO:INFO:SPI INIT\r\n");

    PINS_DRV_WritePin(PTA, 9U, 1); //TEST
    OSIF_TimeDelay(1);

    
}

void MLY_UART1_SEND(const char *fmt, ...)
{
    static uint8_t txBuf[128];   
    va_list args;

    memset(txBuf, 0, sizeof(txBuf));
    va_start(args, fmt);
    vsprintf((char *)txBuf, fmt, args); 
    va_end(args);

    UART_SendDataBlocking(&uart_pal1_instance,(uint8_t *)txBuf,strlen((char *)txBuf), TIMEOUT);
}

int _write(int fd, const char *ptr, int len)
{
	(void)fd;
    UART_SendDataBlocking(&uart_pal1_instance, (uint8_t *)ptr, len, TIMEOUT);
    return len;
}
