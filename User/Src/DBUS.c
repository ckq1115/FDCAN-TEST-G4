//
// Created by CaoKangqi on 2026/1/19.
//

#include "DBUS.h"

/************************************************************万能分隔符**************************************************************
 * 	@author:			//  小瑞 COPY FORM 赵澍
 *	@performance:	    //
 *	@parameter:		    //
 *	@time:				//  22-11-21 18:47
 *	@ReadMe:	        //  这个版本有遥控滚轮的控制，但是某些遥控没有办法正确传回滚轮数据
                        //	遥控通道说明图
                        //	^						^
                        //	|						|
                        //	|ch2---->		        |ch0---->
                        //	|						|
                        //	ch3					    ch1
                        //  共用体接收
 ************************************************************万能分隔符**************************************************************/
void DBUS_Resolved(uint8_t* Data, DBUS_Typedef *DBUS,    DBUS_UNION_Typdef *DBUS_UNION)
{
    DBUS->DBUS_ONLINE_JUDGE_TIME = DBUS_OFFLINE_TIME;

    static uint8_t Key_Q_Lock = 0; // 0是开锁，1是上锁
    static uint8_t Key_E_Lock = 0;
    static uint8_t Key_R_Lock = 0;
    static uint8_t Key_F_Lock = 0;
    static uint8_t Key_G_Lock = 0;
    static uint8_t Key_Z_Lock = 0;
    static uint8_t Key_X_Lock = 0;
    static uint8_t Key_C_Lock = 0;
    static uint8_t Key_V_Lock = 0;
    static uint8_t Key_B_Lock = 0;
    static uint8_t Key_Shift_Lock = 0;
    static uint8_t Key_Ctrl_Lock = 0;
    static uint8_t Mouse_R_Lock = 0;
    static uint8_t Mouse_L_Lock = 0;

    memcpy(DBUS_UNION->GetData , Data , 19);

    DBUS->Remote.S1_u8 = DBUS_UNION->DataNeaten.S1;
    DBUS->Remote.S2_u8 = DBUS_UNION->DataNeaten.S2;

    DBUS->Remote.CH0_int16  = DBUS_UNION->DataNeaten.CH0 -1024;
    DBUS->Remote.CH1_int16  = DBUS_UNION->DataNeaten.CH1 -1024;
    DBUS->Remote.CH2_int16  = DBUS_UNION->DataNeaten.CH2 -1024;
    DBUS->Remote.CH3_int16  = DBUS_UNION->DataNeaten.CH3 -1024;
    DBUS->Remote.Dial_int16 = DBUS_UNION->DataNeaten.Direction -1024;

    if (DBUS_UNION->DataNeaten.CH0 == 0)
    {
        DBUS->Remote.CH0_int16  = 0;
        DBUS->Remote.CH1_int16  = 0;
        DBUS->Remote.CH2_int16  = 0;
        DBUS->Remote.CH3_int16  = 0;
        DBUS->Remote.Dial_int16 = 0;
    }

    //*对点按和长按的区分*//
    DBUS->Mouse.R_State = KEY_STATUS(DBUS_UNION->DataNeaten.Mouse_R , DBUS->Mouse.R_PressTime);
    DBUS->Mouse.L_State = KEY_STATUS(DBUS_UNION->DataNeaten.Mouse_L , DBUS->Mouse.L_PressTime);

    //*键盘的按键*//
    DBUS->KeyBoard.W = KEY_STATUS(DBUS_UNION->DataNeaten.KeyBoard_W , DBUS->KeyBoard.W_PressTime);
    DBUS->KeyBoard.A = KEY_STATUS(DBUS_UNION->DataNeaten.KeyBoard_A , DBUS->KeyBoard.A_PressTime);
    DBUS->KeyBoard.S = KEY_STATUS(DBUS_UNION->DataNeaten.KeyBoard_S , DBUS->KeyBoard.S_PressTime);
    DBUS->KeyBoard.D = KEY_STATUS(DBUS_UNION->DataNeaten.KeyBoard_D , DBUS->KeyBoard.D_PressTime);

    //*鼠标滤波*//
    DBUS->Mouse.X_Flt = OneFilter(DBUS->Mouse.X_Flt , (float) DBUS_UNION->DataNeaten.Mouse_X , 500);
    DBUS->Mouse.Y_Flt = OneFilter(DBUS->Mouse.Y_Flt , (float) DBUS_UNION->DataNeaten.Mouse_Y , 500);

    // Shift
    DBUS->KeyBoard.Shift = DBUS_UNION->DataNeaten.KeyBoard_Shift;
    if (DBUS->KeyBoard.Shift == 1 && Key_Shift_Lock == 0)
    {
        DBUS->KeyBoard.Shift_PreeNumber = !DBUS->KeyBoard.Shift_PreeNumber;
        Key_Shift_Lock = 1; // 上锁
    }
    else if (DBUS->KeyBoard.Shift == 0 && Key_Shift_Lock == 1)
    {
        Key_Shift_Lock = 0; // 开锁
    }
    // Ctrl
    DBUS->KeyBoard.Ctrl = DBUS_UNION->DataNeaten.KeyBoard_Ctrl;
    if (DBUS->KeyBoard.Ctrl == 1 && Key_Ctrl_Lock == 0)
    {
        DBUS->KeyBoard.Ctrl_PreeNumber = !DBUS->KeyBoard.Ctrl_PreeNumber;
        Key_Ctrl_Lock = 1; // 上锁
    }
    else if (DBUS->KeyBoard.Ctrl == 0 && Key_Ctrl_Lock == 1)
    {
        Key_Ctrl_Lock = 0; // 开锁
    }
    // Q
    DBUS->KeyBoard.Q = DBUS_UNION->DataNeaten.KeyBoard_Q;
    if (DBUS->KeyBoard.Q == 1 && Key_Q_Lock == 0)
    {
        DBUS->KeyBoard.Q_PreeNumber = !DBUS->KeyBoard.Q_PreeNumber;
        Key_Q_Lock = 1; // 上锁
    }
    else if (DBUS->KeyBoard.Q == 0 && Key_Q_Lock == 1)
    {
        Key_Q_Lock = 0; // 开锁
    }
    // E
    DBUS->KeyBoard.E = DBUS_UNION->DataNeaten.KeyBoard_E;
    if (DBUS->KeyBoard.E == 1 && Key_E_Lock == 0)
    {
        DBUS->KeyBoard.E_PreeNumber = !DBUS->KeyBoard.E_PreeNumber;
        Key_E_Lock = 1; // 上锁
    }
    else if (DBUS->KeyBoard.E == 0 && Key_E_Lock == 1)
    {
        Key_E_Lock = 0; // 开锁
    }
    // R
    DBUS->KeyBoard.R = DBUS_UNION->DataNeaten.KeyBoard_R;
    if (DBUS->KeyBoard.R == 1 && Key_R_Lock == 0)
    {
        DBUS->KeyBoard.R_PreeNumber = !DBUS->KeyBoard.R_PreeNumber;
        Key_R_Lock = 1; // 上锁
    }
    else if (DBUS->KeyBoard.R == 0 && Key_R_Lock == 1)
    {
        Key_R_Lock = 0; // 开锁
    }
    // F
    DBUS->KeyBoard.F = DBUS_UNION->DataNeaten.KeyBoard_F;
    if (DBUS->KeyBoard.F == 1 && Key_F_Lock == 0)
    {
        DBUS->KeyBoard.F_PreeNumber = !DBUS->KeyBoard.F_PreeNumber;
        Key_F_Lock = 1; // 上锁
    }
    else if (DBUS->KeyBoard.F == 0 && Key_F_Lock == 1)
    {
        Key_F_Lock = 0; // 开锁
    }
    // Z
    DBUS->KeyBoard.Z = DBUS_UNION->DataNeaten.KeyBoard_Z;
    if (DBUS->KeyBoard.Z == 1 && Key_Z_Lock == 0)
    {
        DBUS->KeyBoard.Z_PreeNumber = !DBUS->KeyBoard.Z_PreeNumber;
        Key_Z_Lock = 1; // 上锁
    }
    else if (DBUS->KeyBoard.Z == 0 && Key_Z_Lock == 1)
    {
        Key_Z_Lock = 0; // 开锁
    }
    // G
    DBUS->KeyBoard.G = DBUS_UNION->DataNeaten.KeyBoard_G;
    if (DBUS->KeyBoard.G == 1 && Key_G_Lock == 0)
    {
        DBUS->KeyBoard.G_PreeNumber = !DBUS->KeyBoard.G_PreeNumber;
        Key_G_Lock = 1; // 上锁
    }
    else if (DBUS->KeyBoard.G == 0 && Key_G_Lock == 1)
    {
        Key_G_Lock = 0; // 开锁
    }
    // X
    DBUS->KeyBoard.X = DBUS_UNION->DataNeaten.KeyBoard_X;
    if (DBUS->KeyBoard.X == 1 && Key_X_Lock == 0)
    {
        DBUS->KeyBoard.X_PreeNumber = !DBUS->KeyBoard.X_PreeNumber;
        Key_X_Lock = 1; // 上锁
    }
    else if (DBUS->KeyBoard.X == 0 && Key_X_Lock == 1)
    {
        Key_X_Lock = 0; // 开锁
    }
    // C
    DBUS->KeyBoard.C = DBUS_UNION->DataNeaten.KeyBoard_C;
    if (DBUS->KeyBoard.C == 1 && Key_C_Lock == 0)
    {
        DBUS->KeyBoard.C_PreeNumber = !DBUS->KeyBoard.C_PreeNumber;
        Key_C_Lock = 1; // 上锁
    }
    else if (DBUS->KeyBoard.C == 0 && Key_C_Lock == 1)
    {
        Key_C_Lock = 0; // 开锁
    }

    DBUS->KeyBoard.V = DBUS_UNION->DataNeaten.KeyBoard_V;
	if (DBUS->KeyBoard.V == 1 && Key_V_Lock == 0)
    {
        DBUS->KeyBoard.V_PreeNumber = !DBUS->KeyBoard.V_PreeNumber;
        Key_V_Lock = 1; // 上锁
    }
    else if (DBUS->KeyBoard.V == 0 && Key_V_Lock == 1)
    {
        Key_V_Lock = 0; // 开锁
    }

    DBUS->KeyBoard.B = DBUS_UNION->DataNeaten.KeyBoard_B;
	if (DBUS->KeyBoard.B == 1 && Key_B_Lock == 0)
    {
        DBUS->KeyBoard.B_PreeNumber = !DBUS->KeyBoard.B_PreeNumber;
        Key_B_Lock = 1; // 上锁
    }
    else if (DBUS->KeyBoard.B == 0 && Key_B_Lock == 1)
    {
        Key_B_Lock = 0; // 开锁
    }

	DBUS->Mouse.L_State = DBUS_UNION->DataNeaten.Mouse_L;
	if (DBUS->Mouse.L_State == 1 && Mouse_L_Lock == 0)
    {
        DBUS->Mouse.L_PressTime = !DBUS->Mouse.L_PressTime;
        Mouse_L_Lock = 1; // 上锁
    }
    else if (DBUS->Mouse.L_State == 0 && Mouse_L_Lock == 1)
    {
        Mouse_L_Lock = 0; // 开锁
    }

	DBUS->Mouse.R_State = DBUS_UNION->DataNeaten.Mouse_R;
	if (DBUS->Mouse.R_State == 1 && Mouse_R_Lock == 0)
    {
        DBUS->Mouse.R_PressTime = !DBUS->Mouse.R_PressTime;
        Mouse_R_Lock = 1; // 上锁
    }
    else if (DBUS->Mouse.R_State == 0 && Mouse_R_Lock == 1)
    {
        Mouse_R_Lock = 0; // 开锁
    }
}


/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞 COPY 赵澍
 *	@performance:	    //鼠标滤波
 *	@parameter:		    //上一次的值//当前值//尖峰敏感度
 *	@time:				//22-11-23 16:39
 *	@ReadMe:			//使用一阶低通滤波(改进版)
                        //尖峰敏感度：越小对尖峰越敏感	(一般取值为最大值的20%)
 ************************************************************万能分隔符**************************************************************/
float OneFilter(float last , float now , float thresholdValue)
{
    // 减小平滑滤波值会增大对于细小毛刺的过滤程度
    // 增加尖峰滤波值会增大对于尖峰数值的响应程度
    const float sensitivlFilter = 0.8f; // 尖峰滤波值//小于1
    const float numbFilter = 0.2f; // 平滑滤波值//小于1

    if (MATH_ABS_float(MATH_ABS_float(last) - MATH_ABS_float(now)) >= thresholdValue)
    {
        return (float) (now * sensitivlFilter + last * (1 - sensitivlFilter));
    }
    else
    {
        return (float) (now * numbFilter + last * (1 - numbFilter));
    }
}
/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //按键长按短按
 *	@parameter:		    //
 *	@time:				//23-04-26 21:40
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
uint8_t KEY_STATUS(uint64_t  KEY , uint8_t PRESS_TIME)
{
    if (KEY == 1)
    {
        if (PRESS_TIME <= 20)
        {
            PRESS_TIME++;
            return RUI_DF_KEY_CLICK;
        }
        else
        {
            return  RUI_DF_KEY_PRESS; // 长按
        }
    }
    else
    {
        PRESS_TIME = 0;
        return  RUI_DF_KEY_UP;
    }

}

