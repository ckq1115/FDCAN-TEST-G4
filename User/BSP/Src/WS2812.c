//
// Created by CaoKangqi on 2026/1/23.
//
#include "WS2812.h"
#include <string.h>
#include "tim.h"
#include "BSP_DWT.h"
// 增加 50 个 0 作为复位信号
#define PWM_BUF_LEN (MAX_LED * 24 + WS2812_RESET_LEN)

static WS2812_Color_t LED_Data[MAX_LED];
static uint16_t PWM_Buffer[PWM_BUF_LEN];
static uint8_t Global_Brightness = 255;
static volatile uint8_t isSending = 0;

void WS2812_Init(void) {
    isSending = 0;
    memset(PWM_Buffer, 0, sizeof(PWM_Buffer));
    WS2812_Clear();
    WS2812_Send();
}

// 增加亮度缩放逻辑
void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= MAX_LED) return;

    // 实时计算亮度缩放，避免修改原始数据缓冲区
    if (Global_Brightness != 255) {
        r = (r * Global_Brightness) >> 8;
        g = (g * Global_Brightness) >> 8;
        b = (b * Global_Brightness) >> 8;
    }

    LED_Data[index].R = r;
    LED_Data[index].G = g;
    LED_Data[index].B = b;
}

void WS2812_SetBrightness(uint8_t brightness) {
    Global_Brightness = brightness;
}

void WS2812_Clear(void) {
    memset(LED_Data, 0, sizeof(LED_Data));
}

void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b) {
    for (uint16_t i = 0; i < MAX_LED; i++) {
        WS2812_SetPixel(i, r, g, b);
    }
}

#include <math.h>

/**
 * @brief  指定单灯呼吸逻辑（只负责计算亮度并填入 PWM 缓冲区）
 * @param  index:  LED 索引
 * @param  period: 呼吸周期（秒）
 */
void WS2812_UpdateBreathing(uint16_t index, float period) {
    if (index >= MAX_LED || period <= 0.0f) return;

    // 1. 获取当前时间（DWT 高精度时间轴）
    float currentTime = DWT_GetTimeline_s();

    // 2. 计算当前亮度的比例因子 (0.0 ~ 1.0)
    // 使用 cosf 偏移可以从最亮开始，或者 sinf 从灭开始
    float factor = (sinf(2.0f * 3.1415926f * currentTime / period) + 1.0f) * 0.5f;

    // 3. 从 LED_Data 获取基准颜色，并应用呼吸因子和全局亮度
    // 这里直接计算出该灯的临时 GRB 值
    uint8_t r = (uint8_t)(LED_Data[index].R * factor);
    uint8_t g = (uint8_t)(LED_Data[index].G * factor);
    uint8_t b = (uint8_t)(LED_Data[index].B * factor);

    // 4. 将计算后的颜色实时填入 PWM_Buffer 对应的位置
    // 每一个 LED 占用 24 个 uint16_t
    uint32_t pos = index * 24;
    uint32_t color = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;

    for (int8_t j = 23; j >= 0; j--) {
        PWM_Buffer[pos++] = (color & (1 << j)) ? WS2812_PWM_HIGH : WS2812_PWM_LOW;
    }
}

/**
 * @brief  触发 DMA 发送（修改版的 Send，不再重新计算所有颜色，只负责发出去）
 * @note   在调用此函数前，确保已经通过 UpdateBreathing 或普通转换填充了 PWM_Buffer
 */
void WS2812_Submit(void) {
    if (isSending) return;
    isSending = 1;
    HAL_TIM_PWM_Start_DMA(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL, (uint32_t *)PWM_Buffer, PWM_BUF_LEN);
}

void WS2812_Send(void) {
    if (isSending) return;

    uint32_t pos = 0;
    for (int i = 0; i < MAX_LED; i++) {
        // 提取 GRB 数据
        uint32_t color = ((uint32_t)LED_Data[i].G << 16) |
                         ((uint32_t)LED_Data[i].R << 8)  |
                          (uint32_t)LED_Data[i].B;

        // 展开循环优化性能
        for (int8_t j = 23; j >= 0; j--) {
            PWM_Buffer[pos++] = (color & (1 << j)) ? WS2812_PWM_HIGH : WS2812_PWM_LOW;
        }
    }

    // 确保 Reset 区间为 0
    // 注意：最后 50 个元素在 Init 时已清零，且 Send 过程中不会被触碰

    isSending = 1;
    HAL_TIM_PWM_Start_DMA(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL, (uint32_t *)PWM_Buffer, PWM_BUF_LEN);
}

/**
 * @brief HSV转RGB工具函数 (内部使用)
 * @param h 色调 [0, 255]
 * @param s 饱和度 [0, 255]
 * @param v 亮度 [0, 255]
 */
static void HSV_To_RGB(uint8_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b) {
    uint8_t region, remainder, p, q, t;

    if (s == 0) {
        *r = *g = *b = v;
        return;
    }

    region = h / 43;
    remainder = (h % 43) * 6;

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
        case 0: *r = v; *g = t; *b = p; break;
        case 1: *r = q; *g = v; *b = p; break;
        case 2: *r = p; *g = v; *b = t; break;
        case 3: *r = p; *g = q; *b = v; break;
        case 4: *r = t; *g = p; *b = v; break;
        default: *r = v; *g = p; *b = q; break;
    }
}

/**
 * @brief 四灯幻彩流水灯
 * @param speed 演变速度 (建议调用间隔 5-20ms)
 */
void WS2812_RainbowCycle(uint8_t speed) {
    static uint8_t j = 0; // 色调偏移量
    uint8_t r, g, b;

    // 我们只处理前 4 个 LED
    for (int i = 0; i < MAX_LED; i++) {
        // 每个灯的色调偏移 255/MAX_LED
        HSV_To_RGB(j + (i * 64), 255, 50, &r, &g, &b);
        WS2812_SetPixel(i, r, g, b); //
    }

    j += speed; // 增加偏移量，改变下一帧颜色
    WS2812_Send(); // 发送数据到硬件
}

/**
 * @brief 单色跑马灯功能
 * @param r, g, b 跑马灯的颜色
 * @param speed 移动速度 (建议调用间隔 50-100ms)
 */
void WS2812_SingleRunningLight(uint8_t r, uint8_t g, uint8_t b, uint16_t delay_ms) {
    static uint16_t current_pos = 0; // 记录当前点亮的 LED 位置
    //先清空所有灯的颜色数据
    memset(LED_Data, 0, sizeof(LED_Data));
    //设置当前位置的 LED 颜色
    WS2812_SetPixel(current_pos, r, g, b);
    //发送数据到硬件
    WS2812_Send();
    //更新位置，准备下一帧
    current_pos++;
    if (current_pos >= MAX_LED) {
        current_pos = 0; // 跑完一圈后回到起点
    }
    //控制移动频率
    HAL_Delay(delay_ms);
}

/**
 * @brief 带拖尾效果的单色跑马灯
 * @param r, g, b   跑马灯头部的颜色
 * @param decay     衰减系数 (1-255，数值越小尾巴越长，建议 180-220)
 * @param delay_ms  移动延时 (毫秒)，数值越小速度越快
 */
void WS2812_TrailingRunningLight(uint8_t r, uint8_t g, uint8_t b, uint8_t decay, uint16_t delay_ms) {
    static uint16_t head = 0; // 记录头部位置
    //遍历所有 LED，对当前颜色进行衰减处理
    for (int i = 0; i < MAX_LED; i++) {
        // 计算方式：当前亮度 * decay / 256
        LED_Data[i].R = (LED_Data[i].R * decay) >> 8;
        LED_Data[i].G = (LED_Data[i].G * decay) >> 8;
        LED_Data[i].B = (LED_Data[i].B * decay) >> 8;
    }
    //设置“头部”灯光的最高亮度
    // 即使 head 位置之前有残余亮度，这里也会被重新置为最亮
    WS2812_SetPixel(head, r, g, b);
    //将计算好的颜色数据转换并发送
    WS2812_Send();
    //更新头部位置，实现循环
    head = (head + 1) % MAX_LED;

    // 5. 自定义延时
    HAL_Delay(delay_ms);
}

// 改进的回调函数
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == WS2812_TIM_HANDLE.Instance) {
        // 停止 DMA 防止持续输出最后一条占空比数据
        HAL_TIM_PWM_Stop_DMA(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL);
        isSending = 0;
    }
}