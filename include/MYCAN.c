#include "Cpu.h"
#include "USERINIT.h"
#include "can_pal1.h"          // S32DS ���ɵ����ͷ�ļ�
#include "can_pal.h"
#include "MYCAN.H"

can_message_t txMsg = {0};    // ���ͻ�����
can_message_t rxMsg = {0};    // ���ջ�����
volatile bool rxComplete = false;

extern const can_instance_t can_pal1_instance;

#define FLEXCAN1_BASE          CAN1


can_buff_config_t Rx_buffCfg =  {
        .enableFD = false,
        .enableBRS = false,
        .fdPadding = 0U,
        .idType = CAN_MSG_ID_STD,
        .isRemote = false
    };

can_buff_config_t Tx_buffCfg =  {
    .enableFD = false,
    .enableBRS = false,
    .fdPadding = 0U,
    .idType = CAN_MSG_ID_STD,
    .isRemote = false
};



void CAN1_RxTxCallback(uint32_t instance,
                       can_event_t event,
                       uint32_t buffIdx,
                       void *flexcanState)
                       
{
    (void)instance;
    (void)buffIdx;
    (void)flexcanState;
    // MLY_UART1_SEND("INFO:CAN_CALLBACK USEING\r\n");

    //    �¼�ֵ���ձ�
    MLY_UART1_SEND("CB:inst=%d,evt=%d,buf=%d\r\n", instance, event, buffIdx);
    //    0 = RX_COMPLETE
    //    1 = TX_COMPLETE
    //    2 = RX_FIFO_COMPLETE
    //    3 = BUSOFF
    //    4 = ERROR

    if (event == CAN_EVENT_RX_COMPLETE)
    {
        
        MLY_UART1_SEND("INFO:CAN_EVENT_RX_COMPLETE over \r\n");
    	MLY_UART1_SEND("Loopback OK!  ID=%03X  len=%d  data=%02X %02X %02X %02X\r\n",
                       rxMsg.id,
                       rxMsg.length,
                       rxMsg.data[0],
                       rxMsg.data[1],
                       rxMsg.data[2],
                       rxMsg.data[3]);
    	if (rxMsg.length >= 4)
    		{
    				if (memcmp((char*)rxMsg.data, "#L11!", 5) == 0) LED1_FLAG = 1;
    				if (memcmp((char*)rxMsg.data, "#L10!", 5) == 0) LED1_FLAG = 0;
    		}
    	CAN_Receive(&can_pal1_instance, RX_MAILBOX, &rxMsg);
    	MLY_UART1_SEND("INFO:CANRX CB over\r\n");
        
    }
}

void APP_CAN_Init(void)
{
    MLY_UART1_SEND("INFO:CAN_INIT beginning\r\n");
    //ISR
    INT_SYS_SetPriority(CAN1_ORed_0_15_MB_IRQn, 4); 
    INT_SYS_EnableIRQ(CAN1_ORed_0_15_MB_IRQn);
    MLY_UART1_SEND("INFO:CAN_ISR OK\r\n");
    //Init
    CAN_Init(&can_pal1_instance, &can_pal1_Config0);
    CAN_ConfigRxBuff(&can_pal1_instance, RX_MAILBOX, &Rx_buffCfg, Rx_Filter); //ע��������ú�MSGID������(�����������Ϊ0x1����ֻ����msgid 0x1�����ı���)
    CAN_ConfigTxBuff(&can_pal1_instance, TX_MAILBOX, &Tx_buffCfg); //���÷���
    /*����MSGID�����룬������Կ�������Ϊ��11bit MSGID��ַ�Ĺ���
    ���ĳbitλ��Ҫ��������Ϊ1,����������Ϊ0,������������Ϊ0x7ff�����ȫ����׼id,�������Ϊ0x7fe,��ֻ�ܽ���0x01�ı���(������0x0�ĵ�ַ)*/
    CAN_SetRxFilter(&can_pal1_instance,CAN_MSG_ID_STD,RX_MAILBOX,RX_MASK); //����MSGID���룬

    MLY_UART1_SEND("INFO:CAN_InstallEventCallback beginning\r\n");
    CAN_InstallEventCallback(&can_pal1_instance,CAN1_RxTxCallback,NULL);
    MLY_UART1_SEND("INFO:CAN_InstallEventCallback over\r\n");

    //     /* �� MB0 ��Ϊ��ֻ�� 0x123�� */
    // APP_CAN_SetRxMask(); 
    /*��������*/
    CAN_Receive(&can_pal1_instance, RX_MAILBOX, &rxMsg);
   
    MLY_UART1_SEND("can_pal1_instance @ %p\r\n", &can_pal1_instance);
    MLY_UART1_SEND("instType = %d\r\n", can_pal1_instance.instType);
    MLY_UART1_SEND("APP_CAN_Init Over\r\n");
    
}

// status_t APP_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len)
// {
//     txMsg.id       = id;
//     txMsg.length   = len;
//     for (uint8_t i = 0; i < len; i++) txMsg.data[i] = data[i];

//     return CAN_Send(&can_pal1_instance, TX_MAILBOX, &txMsg);
// }

void APP_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len)
{
    txMsg.id       = id;
    txMsg.length   = len;
    for (uint8_t i = 0; i < len; i++) txMsg.data[i] = data[i];
    CAN_Send(&can_pal1_instance, TX_MAILBOX, &txMsg);


}

//TMD �ܲ����� ֱ�ӻػ�����
void APP_CAN_LoopbackTest(void)
{
    static uint32_t cnt = 0;
    // if (cnt == 0)MLY_UART1_SEND("Loopback Startr\r\n");

    /* �������� 0x00 -> 0x63 */
    uint8_t LoopbackTXdate[8] = {cnt, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
    APP_CAN_Send(0x123U, LoopbackTXdate, 8);          /* ����һ֡ */
    // cnt++;
    // if (cnt >= 100000) cnt = 0; 
}
