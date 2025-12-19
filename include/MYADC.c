#include "Cpu.h"
#include "USERINIT.h"
#include "MYADC.h"

float adcMax = 65535.0f;  // 16 位 ADC 最大值
#define ADC_VREFH 3.3f    // 参考电压高 (3.3V，确认 VDDA)
#define ADC_VREFL 0.0f    // 参考电压低 (GND)
uint16_t adcRawValue;     // ADC 原始值 (16 位)
uint16_t adcRawValue;
 float ADC0_CH16_READ(void)
  {
	  float adcValue;
      ADC_DRV_ConfigChan(INST_ADCONV1, 0U, &adConv1_ChnConfig0);
      ADC_DRV_WaitConvDone(INST_ADCONV1);
      ADC_DRV_GetChanResult(INST_ADCONV1, 0U, &adcRawValue);
      adcValue = ((float) adcRawValue / adcMax) * (ADC_VREFH - ADC_VREFL);
      return adcValue;
  }
