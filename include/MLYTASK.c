#include "USERINIT.h"
#include "MLYTASK.h"
uint8_t test;

uint8_t LED1_FLAG,LED2_FLAG;
uint8_t KEY_VAL,KEY_OLD,KEY_DOWN,KEY_UP;


void LED_PROC(void);
void KEY_PROC(void);
void ADC_PROC(void);
void ADS7953_PROC(void);

void MLYTASK(void)
{
    // LED_PROC();
    // KEY_PROC();
    ADS7953_PROC();
    // APP_CAN_LoopbackTest();
}

void LED_PROC(void)
{
	if(LED1_FLAG==1)
	{
        LED1_ON();
	}
	else if(LED1_FLAG==2)
	{
        LED1_OFF();
	}
}
void KEY_PROC(void)
{
    KEY_VAL=KEY_READ();
    KEY_DOWN=KEY_VAL&(KEY_VAL^KEY_OLD);
    KEY_UP=~KEY_VAL&(KEY_VAL^KEY_OLD);
    KEY_OLD=KEY_VAL;
        switch (KEY_VAL)
        {
            case 1:
            	MLY_UART1_SEND("MLY_DEBUG_ KEY 1 OK && LED1_ON\r\n");
                LED1_FLAG=1;
                test=1;
                break;
            case 2:
            	MLY_UART1_SEND("MLY_DEBUG_ KEY 2 && LED1_OF OK\r\n");
            	LED1_FLAG=2;
            	test=2;

                break;
        }
    // }
}
void ADC_PROC(void)
{
    float ADC_VAL;
    ADC_VAL=ADC0_CH16_READ();
    MLY_UART1_SEND("%f\r\n",ADC_VAL);



}

void ADS7953_PROC(void)
{
    ADS7953_Scan();
}