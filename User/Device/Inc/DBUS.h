//
// Created by CaoKangqi on 2026/1/19.
//

#ifndef FDCAN_TEST_G4_DBUS_H
#define FDCAN_TEST_G4_DBUS_H

#include "stdint.h"
#include "CKQ_MATH.h"
#include "string.h"
#include "gpio.h"
#include "usart.h"

#ifndef PassState

#define RUI_DF_KEY_CLICK     1 // 点按
#define RUI_DF_KEY_UP        0 // 松开
#define RUI_DF_KEY_PRESS     2 // 长按

#endif

#define RUI_Mouse_VX RUI_V_DBUS.Mouse.X_Flt // 鼠标横向速度
#define RUI_Mouse_VY RUI_V_DBUS.Mouse.Y_Flt // 鼠标纵向速度
#define RUI_Mouse_L  RUI_V_DBUS.Mouse.R_State
#define RUI_Mouse_R  RUI_V_DBUS.Mouse.L_State
//按下为1松开为0
#define RUI_KeyBoard_A     RUI_V_DBUS.KeyBoard.A     // a键调用
#define RUI_KeyBoard_S     RUI_V_DBUS.KeyBoard.S     // s键调用
#define RUI_KeyBoard_D     RUI_V_DBUS.KeyBoard.D     // d键调用
#define RUI_KeyBoard_W     RUI_V_DBUS.KeyBoard.W     // w键调用
#define RUI_KeyBoard_Q     RUI_V_DBUS.KeyBoard.Q     // q键调用
#define RUI_KeyBoard_E     RUI_V_DBUS.KeyBoard.E     // e键调用
#define RUI_KeyBoard_R     RUI_V_DBUS.KeyBoard.R     // r键调用
#define RUI_KeyBoard_F     RUI_V_DBUS.KeyBoard.F     // r键调用
#define RUI_KeyBoard_T     RUI_V_DBUS.KeyBoard.T     // t键调用
#define RUI_KeyBoard_G     RUI_V_DBUS.KeyBoard.G     // g键调用
#define RUI_KeyBoard_Z     RUI_V_DBUS.KeyBoard.Z     // z键调用
#define RUI_KeyBoard_X     RUI_V_DBUS.KeyBoard.X     // x键调用
#define RUI_KeyBoard_C     RUI_V_DBUS.KeyBoard.C     // c键调用
#define RUI_KeyBoard_V     RUI_V_DBUS.KeyBoard.V     // v键调用
#define RUI_KeyBoard_B     RUI_V_DBUS.KeyBoard.B     // b键调用
#define RUI_KeyBoard_Ctrl  RUI_V_DBUS.KeyBoard.Ctrl  // Ctrl键调用
#define RUI_KeyBoard_Shift RUI_V_DBUS.KeyBoard.Shift // shift键调用
// 按下为1，再次按下为0
#define RUI_KeyBoard_Q_Lock     RUI_V_DBUS.KeyBoard.Q_PreeNumber
#define RUI_KeyBoard_E_Lock     RUI_V_DBUS.KeyBoard.E_PreeNumber
#define RUI_KeyBoard_R_Lock     RUI_V_DBUS.KeyBoard.R_PreeNumber
#define RUI_KeyBoard_F_Lock     RUI_V_DBUS.KeyBoard.F_PreeNumber
#define RUI_KeyBoard_G_Lock     RUI_V_DBUS.KeyBoard.G_PreeNumber
#define RUI_KeyBoard_Z_Lock     RUI_V_DBUS.KeyBoard.Z_PreeNumber
#define RUI_KeyBoard_X_Lock     RUI_V_DBUS.KeyBoard.X_PreeNumber
#define RUI_KeyBoard_C_Lock     RUI_V_DBUS.KeyBoard.C_PreeNumber
#define RUI_KeyBoard_V_Lock     RUI_V_DBUS.KeyBoard.V_PreeNumber
#define RUI_KeyBoard_B_Lock     RUI_V_DBUS.KeyBoard.B_PreeNumber
#define RUI_KeyBoard_Shift_Lock RUI_V_DBUS.KeyBoard.Shift_PreeNumber
#define RUI_KeyBoard_Ctrl_Lock  RUI_V_DBUS.KeyBoard.Ctrl_PreeNumber

typedef struct
{
    int8_t ONLINE_JUDGE_TIME; // 在线检测时间
    //遥控
    struct
    {
        int16_t CH0_int16;
        int16_t CH1_int16;
        int16_t CH2_int16;
        int16_t CH3_int16;
        int16_t Dial_int16;
        uint8_t S1_u8;
        uint8_t S2_u8;
        int8_t Error_int8;
    } Remote;
    //鼠标
    struct
    {
        float X_Flt;
        float X_Filter; // 处理后使用的鼠标值
        float X_Max;    // 记录鼠标的最大值用来校准鼠标
        float Y_Flt;
        float Y_Filter; // 处理后使用的鼠标值
        float Y_Max;    // 记录鼠标的最大值用来校准鼠标
        float Z_Flt;
        float Z_Filter; // 处理后使用的鼠标值
        float Z_Max;    // 记录鼠标的最大值用来校准鼠标
        uint8_t R_State : 4;
        uint8_t L_State : 4;
        uint8_t R_PressTime;
        uint8_t L_PressTime;
    } Mouse;
    //键盘
    struct
    {
        uint8_t W : 4;
        uint8_t S : 4;
        uint8_t A : 4;
        uint8_t D : 4;
        uint8_t Shift : 4;
        uint8_t Ctrl : 4;
        uint8_t Q : 4;
        uint8_t E : 4;
        uint8_t R : 4;
        uint8_t F : 4;
        uint8_t G : 4;
        uint8_t Z : 4;
        uint8_t X : 4;
        uint8_t C : 4;
        uint8_t V : 4;
        uint8_t B : 4;
        uint8_t W_PressTime;
        uint8_t S_PressTime;
        uint8_t A_PressTime;
        uint8_t D_PressTime;
        uint8_t Shift_PreeNumber : 1;
        uint8_t Ctrl_PreeNumber : 1;
        uint8_t Q_PreeNumber : 1;
        uint8_t E_PreeNumber : 1;
        uint8_t R_PreeNumber : 1;
        uint8_t F_PreeNumber : 1;
        uint8_t G_PreeNumber : 1;
        uint8_t Z_PreeNumber : 1;
        uint8_t X_PreeNumber : 1;
        uint8_t C_PreeNumber : 1;
        uint8_t V_PreeNumber : 1;
        uint8_t B_PreeNumber : 1;
    } KeyBoard;
    uint8_t DBUS_ONLINE_JUDGE_TIME;
}DBUS_Typedef;


typedef union  // 使用共用体整合数据
{
    struct
    {
// 遥控数据
        uint64_t CH0 : 11;
        uint64_t CH1 : 11;
        uint64_t CH2 : 11;
        uint64_t CH3 : 11;
        uint64_t S1 : 2;
        uint64_t S2 : 2;
        // 鼠标数据
        int64_t Mouse_X : 16;
        int64_t Mouse_Y : 16;
        int64_t Mouse_Z : 16;
        int64_t Mouse_R : 8;
        int64_t Mouse_L : 8;
        // 键盘数据
        uint64_t KeyBoard_W : 1;
        uint64_t KeyBoard_S : 1;
        uint64_t KeyBoard_A : 1;
        uint64_t KeyBoard_D : 1;
        uint64_t KeyBoard_Shift : 1;
        uint64_t KeyBoard_Ctrl : 1;
        uint64_t KeyBoard_Q : 1;
        uint64_t KeyBoard_E : 1;
        uint64_t KeyBoard_R : 1;
        uint64_t KeyBoard_F : 1;
        uint64_t KeyBoard_G : 1;
        uint64_t KeyBoard_Z : 1;
        uint64_t KeyBoard_X : 1;
        uint64_t KeyBoard_C : 1;
        uint64_t KeyBoard_V : 1;
        uint64_t KeyBoard_B : 1;
        // 遥控滑轮
        uint64_t Direction : 11;
        uint64_t : 0;
    } DataNeaten;
    // 接收到的数组
    uint8_t  GetData[19];
}DBUS_UNION_Typdef;

//遥控接收
void DBUS_Resolved(uint8_t* Data, DBUS_Typedef *RUI_V_DBUS,    DBUS_UNION_Typdef *RUI_V_DBUS_UNION);
// 鼠标滤波
float OneFilter(float last , float now , float thresholdValue);
uint8_t KEY_STATUS(uint64_t  KEY , uint8_t PRESS_TIME);

#endif //FDCAN_TEST_G4_DBUS_H