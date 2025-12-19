#include "Cpu.h"
#include "KEY.h"

#define READ_PIN(PORT, PIN)   ( ((PINS_DRV_ReadPins(PORT) >> (PIN)) & 0x01) )

uint8_t KEY_READ(void)
{
    if (READ_PIN(KEY1_PIN_PORT, KEY1_PIN)==0)   // KEY1 PUSH
    {
        return 1;
    }

    if (READ_PIN(KEY2_PIN_PORT, KEY2_PIN)==0)   // KEY2 PUSH
    {
        return 2;
    }

    return 0;  // NO KEY
}
