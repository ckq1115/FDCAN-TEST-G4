//
// Created by CaoKangqi on 2026/1/23.
//

#ifndef FDCAN_TEST_G4_WS2812_H
#define FDCAN_TEST_G4_WS2812_H

#include "main.h"

#define MAX_LED 4
#define WS2812_TIM_HANDLE htim8
#define WS2812_TIM_CHANNEL TIM_CHANNEL_4


#define WS2812_PWM_LOW  71
#define WS2812_PWM_HIGH 141
#define WS2812_RESET_LEN 50  // Reset 信号所需的低电平数量

typedef struct {
    uint8_t R;
    uint8_t G;
    uint8_t B;
} WS2812_Color_t;

void WS2812_Init(void);
void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);
void WS2812_SetBrightness(uint8_t brightness); // 0-255
void WS2812_Clear(void);
void WS2812_Send(void);
void WS2812_RainbowCycle(uint8_t speed);
void WS2812_SingleRunningLight(uint8_t r, uint8_t g, uint8_t b, uint16_t delay_ms);
void WS2812_TrailingRunningLight(uint8_t r, uint8_t g, uint8_t b, uint8_t decay, uint16_t delay_ms);

#endif //FDCAN_TEST_G4_WS2812_H