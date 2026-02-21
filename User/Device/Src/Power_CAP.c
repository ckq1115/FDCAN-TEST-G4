//
// Created by CaoKangqi on 2026/2/14.
//
#include "Power_CAP.h"

#include "All_define.h"

/************************************************************万能分隔符**************************************************************
 * 	@author:			//瑞
 *	@performance:	    //
 *	@parameter:		    //
 *	@time:				//2024/1/12 16:51
 *	@ReadMe:			//电容接收解算函数
 ************************************************************万能分隔符**************************************************************/
void Power_CAP_CAN_RX(CAPDATE_TYPDEF *CAP_DATA, uint8_t *DATA)
{
    CAP_DATA->GET.CAP_VOLT = (float) ( DATA[ 0 ] << 8 | DATA[ 1 ] );

    CAP_DATA->GET.NOW_POWER = (float) ( DATA[ 2 ] << 8 | DATA[ 3 ] );

    CAP_DATA->GET.NOW_POWER *= 0.01f;

    CAP_DATA->GET.ONLINE_JUDGE_TIME = CAP_OFFLINE_TIME;
}

/*void Power_CAP_CAN_TX(hcan_t* hcan,uint16_t cap_id, CAP_SETDATA *CAP_SET, User_Data_T *User_data)
{
    CAP_SET->CHANNAL.MAX_POWER = (uint8_t)User_data->robot_status.chassis_power_limit;
    CAP_SET->CHANNAL.BUFFER_NOW = (uint8_t)User_data->power_heat_data.buffer_energy;
    //    CAP_SET->CHANNAL.OUT_POWER = (uint8_t)User_data->power_heat_data.chassis_power_reserved;
    CAP_SET->CHANNAL.VOLT = (uint8_t)get_battery_voltage();

    canx_send_data(hcan, cap_id, CAP_SET->SEND_DATA);
}*/