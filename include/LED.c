#include "Cpu.h"
#include "S32K148.h"
#include "pins_driver.h"
#include "pin_mux.h"

#include "string.h"
#include "stdarg.h"
#include "stdio.h"
#include "led.h"
//我们的LED4坏了
void LED1_ON(void)
{
    PINS_DRV_WritePin(LED1_PIN_PORT, LED1_PIN, 0);
}

void LED1_OFF(void)
{
    PINS_DRV_WritePin(LED1_PIN_PORT, LED1_PIN, 1);
}

void LED1_TOGGLE(void)
{
    PINS_DRV_TogglePins(LED1_PIN_PORT, 1<<LED1_PIN);
}

void LED2_ON(void)
{
    PINS_DRV_WritePin(LED2_PIN_PORT, LED2_PIN, 0);
}

void LED2_OFF(void)
{
    PINS_DRV_WritePin(LED2_PIN_PORT, LED2_PIN, 1);
}

void LED2_TOGGLE(void)
{
    PINS_DRV_TogglePins(LED2_PIN_PORT, 1<<LED2_PIN);
}
