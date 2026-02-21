//
// Created by CaoKangqi on 2026/1/23.
//

#ifndef G4_FRAMEWORK_WS2812_H
#define G4_FRAMEWORK_WS2812_H

#include <stdint.h>

#define WS2812_TIM_HANDLE htim8
#define WS2812_TIM_CHANNEL TIM_CHANNEL_4


#define WS2812_PWM_LOW  71
#define WS2812_PWM_HIGH 141

#define MAX_LED 4         // LED 灯珠总数
#define WS2812_RESET_SLOTS 50  // Reset 信号所需的低电平数量

typedef struct {
    uint8_t R;
    uint8_t G;
    uint8_t B;
} WS2812_Color_t;

// 函数声明
void WS2812_Init(void);
void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);
void WS2812_Clear(void);
void WS2812_Send(void); // 启动发送

// 特效函数
void WS2812_UpdateBreathing(uint16_t index, float period);
void WS2812_RainbowCycle(uint8_t speed);

#endif //G4_FRAMEWORK_WS2812_H