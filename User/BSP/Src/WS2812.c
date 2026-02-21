//
// Created by CaoKangqi on 2026/1/23.
//
#include "WS2812.h"

#include <math.h>
#include <string.h>

#include "BSP_DWT.h"
#include "tim.h"

// ================= 查表优化区域 =================

// 0-255 的正弦波表 (0 -> 255 -> 0)，用于呼吸灯，避免 sinf 计算
const uint8_t Sine_Table[256] = {
    0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 15, 17, 18, 20, 22, 24, 26, 28, 30, 32, 35, 37, 39, 42, 44, 47, 49, 52, 55, 58, 60, 63, 66, 69, 72, 75, 78, 81, 85, 88, 91, 94, 97, 101, 104, 107, 111, 114, 117, 121, 124, 127, 131, 134, 137, 141, 144, 147, 150, 154, 157, 160, 163, 167, 170, 173, 176, 179, 182, 185, 188, 191, 194, 197, 200, 202, 205, 208, 210, 213, 215, 217, 220, 222, 224, 226, 229, 231, 232, 234, 236, 238, 239, 241, 242, 244, 245, 246, 248, 249, 250, 251, 251, 252, 253, 253, 254, 254, 255, 255, 255, 255, 255, 255, 255, 254, 254, 253, 253, 252, 251, 251, 250, 249, 248, 246, 245, 244, 242, 241, 239, 238, 236, 234, 232, 231, 229, 226, 224, 222, 220, 217, 215, 213, 210, 208, 205, 202, 200, 197, 194, 191, 188, 185, 182, 179, 176, 173, 170, 167, 163, 160, 157, 154, 150, 147, 144, 141, 137, 134, 131, 127, 124, 121, 117, 114, 111, 107, 104, 101, 97, 94, 91, 88, 85, 81, 78, 75, 72, 69, 66, 63, 60, 58, 55, 52, 49, 47, 44, 42, 39, 37, 35, 32, 30, 28, 26, 24, 22, 20, 18, 17, 15, 13, 12, 11, 9, 8, 7, 6, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0
};

// ================= 变量定义 =================

// 1. 逻辑颜色数据 (保持全量，因为需要随机访问修改)
static WS2812_Color_t Base_Color[MAX_LED];
static WS2812_Color_t LED_Data[MAX_LED];

// 2. 双缓冲 DMA 发送数组
// 只包含 2 个 LED 的数据量 (24 bits * 2 = 48 words)
// 前 24 个是 Buffer_A，后 24 个是 Buffer_B
#define WS2812_DMA_BUF_LEN (24 * 2)
uint16_t DMA_Buffer[WS2812_DMA_BUF_LEN];

static uint8_t Global_Brightness = 255;
static volatile uint8_t isSending = 0;
static uint16_t send_pixel_idx = 0; // 当前正在填充第几个像素的数据

// ================= 内部辅助函数 =================

/**
 * @brief 将一个 LED 的 RGB 数据填充到 DMA 缓冲区的指定位置
 * @param ledIdx: LED_Data 数组中的索引 (如果是复位阶段，则不用)
 * @param bufferOffset: DMA_Buffer 中的偏移 (0 或 24)
 * @param isReset: 是否发送复位信号(0占空比)
 */
static void Fill_Buffer(uint16_t ledIdx, uint16_t bufferOffset, uint8_t isReset) {
    if (isReset) {
        memset(&DMA_Buffer[bufferOffset], 0, 24 * sizeof(uint16_t));
        return;
    }

    // 获取颜色并计算全局亮度
    uint8_t r = LED_Data[ledIdx].R;
    uint8_t g = LED_Data[ledIdx].G;
    uint8_t b = LED_Data[ledIdx].B;

    // 简单的位移计算亮度，比 float 快得多
    if (Global_Brightness < 255) {
        r = (r * Global_Brightness) >> 8;
        g = (g * Global_Brightness) >> 8;
        b = (b * Global_Brightness) >> 8;
    }

    // 组合颜色 GRB (WS2812 标准)
    uint32_t color = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;

    // 填充 24 位 PWM 数据
    for (int8_t i = 23; i >= 0; i--) {
        DMA_Buffer[bufferOffset++] = (color & (1 << i)) ? WS2812_PWM_HIGH : WS2812_PWM_LOW;
    }
}

// ================= 外部接口 =================

void WS2812_Init(void) {
    isSending = 0;
    WS2812_Clear();
    // 这里不需要预填充 PWM Buffer，Send 时会做
}

void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= MAX_LED) return;
    Base_Color[index].R = r;
    Base_Color[index].G = g;
    Base_Color[index].B = b;

    // 初始化显示色，防止没调用 Update 前是黑的
    LED_Data[index] = Base_Color[index];
}

void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b) {
    for (uint16_t i = 0; i < MAX_LED; i++) {
        WS2812_SetPixel(i, r, g, b);
    }
}

void WS2812_Clear(void) {
    memset(LED_Data, 0, sizeof(LED_Data));
}

// 核心发送函数
void WS2812_Send(void) {
    if (isSending) return;

    send_pixel_idx = 0; // 重置发送索引

    // 1. 预填充前两个 LED 的数据到 DMA 缓冲区
    Fill_Buffer(0, 0, 0);  // 填充 Buffer前半部分 (LED 0)
    Fill_Buffer(1, 24, 0); // 填充 Buffer后半部分 (LED 1)

    // 指向下一个要处理的像素
    send_pixel_idx = 2;

    isSending = 1;

    // 2. 启动循环 DMA
    // 注意：这里长度是 48 (WS2812_DMA_BUF_LEN)
    HAL_TIM_PWM_Start_DMA(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL, (uint32_t *)DMA_Buffer, WS2812_DMA_BUF_LEN);
}


// ================= 中断回调 (双缓冲逻辑核心) =================

// 半传输完成 (Half Transfer) -> 刚刚发完了 Buffer 的前半部分 (0-23)
// 此时 DMA 正在发后半部分，CPU 赶紧去填充前半部分
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != WS2812_TIM_HANDLE.Instance) return;

    // 如果还有 LED 数据没发完
    if (send_pixel_idx < MAX_LED) {
        Fill_Buffer(send_pixel_idx, 0, 0); // 填数据到前半段
        send_pixel_idx++;
    }
    // 如果 LED 数据发完了，但是需要发 Reset 信号 (50us 的低电平)
    else if (send_pixel_idx < MAX_LED + WS2812_RESET_SLOTS) {
        Fill_Buffer(0, 0, 1); // 填 0 到前半段
        send_pixel_idx++;
    }
    // 全发完了，不需要在这里停，通常在 TC (全传输) 里停比较整齐，或者让它跑完 reset
}

// 全传输完成 (Transfer Complete) -> 刚刚发完了 Buffer 的后半部分 (24-47)
// 此时 DMA 回滚去发前半部分，CPU 赶紧去填充后半部分
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != WS2812_TIM_HANDLE.Instance) return;

    // 如果还有 LED 数据没发完
    if (send_pixel_idx < MAX_LED) {
        Fill_Buffer(send_pixel_idx, 24, 0); // 填数据到后半段
        send_pixel_idx++;
    }
    // 发 Reset 信号
    else if (send_pixel_idx < MAX_LED + WS2812_RESET_SLOTS) {
        Fill_Buffer(0, 24, 1); // 填 0 到后半段
        send_pixel_idx++;
    }
    // Reset 信号也发够了，停止发送
    else {
        HAL_TIM_PWM_Stop_DMA(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL);
        isSending = 0;

        // 关键：最后手动拉低数据线，防止 PWM 停在高电平烧灯或乱码
        // 具体实现看你的 GPIO 配置，通常 Stop DMA 后 GPIO 会回到初始状态(Reset)
    }
}


// ================= 特效优化 (移除 math.h) =================

/**
 * @brief  让指定灯珠按底色进行呼吸
 * @param  index:  LED 索引
 * @param  period: 呼吸周期 (秒)
 */
void WS2812_UpdateBreathing(uint16_t index, float period) {
    if (index >= MAX_LED) return;
    uint16_t period_ms = (uint16_t)(period * 1000);
    // 使用系统毫秒计数器代替 DWT 浮点时间
    uint32_t now = HAL_GetTick();
    // 将周期映射到 0-255 的查表索引
    uint32_t idx = (now % period_ms) * 255 / period_ms;
    uint32_t factor = Sine_Table[idx];

    // 整数位移代替浮点缩放
    LED_Data[index].R = (Base_Color[index].R * factor) >> 8;
    LED_Data[index].G = (Base_Color[index].G * factor) >> 8;
    LED_Data[index].B = (Base_Color[index].B * factor) >> 8;
}

// 供上层调用的无浮点 HSV 转 RGB (极其高效)
// h: 0-255, s: 0-255, v: 0-255
void WS2812_SetHSV(uint16_t index, uint8_t h, uint8_t s, uint8_t v) {
    uint8_t r, g, b;
    uint8_t region, remainder, p, q, t;

    if (s == 0) {
        r = v; g = v; b = v;
    } else {
        region = h / 43;
        remainder = (h - (region * 43)) * 6;

        p = (v * (255 - s)) >> 8;
        q = (v * (255 - ((s * remainder) >> 8))) >> 8;
        t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

        switch (region) {
            case 0: r = v; g = t; b = p; break;
            case 1: r = q; g = v; b = p; break;
            case 2: r = p; g = v; b = t; break;
            case 3: r = p; g = q; b = v; break;
            case 4: r = t; g = p; b = v; break;
            default: r = v; g = p; b = q; break;
        }
    }
    WS2812_SetPixel(index, r, g, b);
}