#include "USERINIT.h"
#include "MLYTASK.h"

extern uint32_t OSIF_GetMilliseconds(void);

void ADS7953_PROC(void);

void MLYTASK(void)
{
    static uint32_t last_adc_tick = 0U;
    static uint32_t last_led_tick = 0U;
    uint32_t now = OSIF_GetMilliseconds();

    /* ADC scan every 20ms (50Hz) */
    if ((now - last_adc_tick) >= 20U)
    {
        last_adc_tick = now;
        ADS7953_PROC();
    }

    /* LED toggle every 500ms */
    if ((now - last_led_tick) >= 500U)
    {
        last_led_tick = now;
        LED1_TOGGLE();
    }
}

void ADS7953_PROC(void)
{
    /* 调用完整扫描（参考例程） */
    ADS7953_Scan();
}