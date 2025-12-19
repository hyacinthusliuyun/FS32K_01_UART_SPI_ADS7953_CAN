#ifndef LIUYUN_AD7689_H_
#define LIUYUN_AD7689_H_

#include "USERINIT.h"
#include "LiuyunLPSPI2.h"
#include "osif.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== 配置区 ==================== */
#define AD7689_LPSPI_INSTANCE       LPSPICOM2
#define AD7689_PCS                  LPSPI_PCS1          // PTE10

// CNV 引脚（必须用独立 GPIO 控制，不能用 SPI PCS！）
#define AD7689_CNV_PORT             PTE
#define AD7689_CNV_PIN              11U                 // 根据你硬件改
#define AD7689_CNV_H()              PINS_DRV_SetPins(AD7689_CNV_PORT, (1U << AD7689_CNV_PIN))
#define AD7689_CNV_L()              PINS_DRV_ClearPins(AD7689_CNV_PORT, (1U << AD7689_CNV_PIN))

// 参考电压（外部或内部）
#define AD7689_VREF                 5.000f              // 你的实际 VREF

// 建立时间（关键！越大越稳，建议 5~20us）
#define AD7689_ACQ_DELAY_US        10U                 // 10us 绝对稳，压差大时可调到 20us

/* ==================== 数据结构 ==================== */
typedef struct {
    uint8_t   ch;                       // 当前读取的通道号 (0~7)
    uint16_t  raw[8];                   // 原始 16bit 码值
    float     volt[8];                  // 实际电压值 (V)
    float     vin[8];                   // 最终应用电压（按你原来逻辑）
    uint16_t  cfgReadback;              // 配置寄存器回读（调试用）
} LiuyunAD7689_t;

extern LiuyunAD7689_t AD7689;

/* ==================== API ==================== */
/* 初始化（只需调用一次）*/
void AD7689_Init(void);

/* 读取指定通道（最推荐方式，绝对不跳变）*/
uint16_t AD7689_ReadChannel(uint8_t ch);

/* 扫描所有 8 通道（带足够建立时间）*/
void AD7689_ScanAllChannels(void);

#endif /* LIUYUN_AD7689_H_ */