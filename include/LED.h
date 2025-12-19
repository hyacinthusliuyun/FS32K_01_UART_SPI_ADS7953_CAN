#ifndef __LED_H_
#define __LED_H_


#define LED1_PIN_PORT 	PTD
#define LED2_PIN_PORT 	PTD
#define PTC_CLOCK	PCC_PORTD_CLOCK
#define LED1_PIN		13U
#define LED2_PIN		14U


void LED1_ON(void);
void LED1_OFF(void);
void LED1_TOGGLE(void);

void LED2_ON(void);
void LED2_OFF(void);
void LED2_TOGGLE(void);

#endif

