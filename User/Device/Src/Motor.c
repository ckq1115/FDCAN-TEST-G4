//
// Created by CaoKangqi on 2026/1/19.
//
#include "Motor.h"

All_Motor_TypeDef All_Motor;

void MOTOR_CAN_RX_3508RM(DJI_MOTOR_DATA_Typedef* DATA , uint8_t* can_data)
{
    DATA->Angle_last = DATA->Angle_now;

    DATA->Angle_now = (int16_t) (((can_data[0] << 8) | can_data[1]) & 0xFFFF);

    DATA->Speed_last = DATA->Speed_now;

    DATA->Speed_now = (int16_t) (((can_data[2] << 8) | can_data[3]) & 0xFFFF);

    DATA->current   = (int16_t) (((can_data[4] << 8) | can_data[5]) & 0xFFFF);

    DATA->temperature = can_data[6];

    if (DATA->Angle_now - DATA->Angle_last < -4000)
    {
        DATA->Laps++;
    }
    else if (DATA->Angle_now - DATA->Angle_last > 4000)
    {
        DATA->Laps--;
    }

    if ((DATA->Laps > 32500) | (DATA->Laps < -32500))
    {
        DATA->Laps = 0;
        DATA->Aim  = DATA->Angle_now;
    }

    DATA->ONLINE_JUDGE_TIME = 15;

    DATA->Angle_Infinite = (int32_t) ((DATA->Laps << 13)+ DATA->Angle_now);

}

/************************************************************万能分隔符**************************************************************
 *	@performance:	    //2006电机解算函数
 *	@parameter:		    //@电机结构体  @can接收到的数组
 *	@time:				//22-11-23 18:49
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
void MOTOR_CAN_RX_2006RM(DJI_MOTOR_DATA_Typedef* DATA , uint8_t* can_data)
{
    DATA->Angle_last = DATA->Angle_now;

    DATA->Angle_now = (int16_t) (((can_data[0] << 8) | can_data[1]) & 0xFFFF);

    DATA->Speed_last = DATA->Speed_now;

    DATA->Speed_now = (int16_t) (((can_data[2] << 8) | can_data[3]) & 0xFFFF);

    DATA->current = (int16_t) (((can_data[4] << 8) | can_data[5]) & 0xFFFF);

    if (DATA->Angle_now - DATA->Angle_last < -4000)
    {
        DATA->Laps++;
    }
    else if (DATA->Angle_now - DATA->Angle_last > 4000)
    {
        DATA->Laps--;
    }

    if ((DATA->Laps > 32500) | (DATA->Laps < -32500))
    {
        DATA->Laps = 0;
        DATA->Aim  = DATA->Angle_now;
    }

    DATA->Angle_Infinite = (int32_t) ((DATA->Laps << 13) + DATA->Angle_now);
    DATA->ONLINE_JUDGE_TIME = 15;
}

/************************************************************万能分隔符**************************************************************
 *	@performance:	    //6020电机解算函数
 *	@parameter:		    //@电机结构体  @can接收到的数组
 *	@time:				//22-11-23 18:50
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
void MOTOR_CAN_RX_6020RM(DJI_MOTOR_DATA_Typedef* DATA , uint8_t* can_data)
{
    DATA->Angle_last = DATA->Angle_now;

    DATA->Angle_now = (int16_t) (((can_data[0] << 8) | can_data[1]) & 0xFFFF);

    DATA->Speed_last = DATA->Speed_now;

    DATA->Speed_now = (int16_t) (((can_data[2] << 8) | can_data[3]) & 0xFFFF);

    DATA->current   = (int16_t) (((can_data[4] << 8) | can_data[5]) & 0xFFFF);

    DATA->temperature = can_data[6];

//    if(DATA->Angle_now < 2900)	DATA->Laps = 1;
//    if(DATA->Angle_now > 4000)	DATA->Laps = 0;

    if (DATA->Angle_now - DATA->Angle_last < -4000)
    {
        DATA->Laps++;
    }
    else if (DATA->Angle_now - DATA->Angle_last > 4000)
    {
        DATA->Laps--;
    }

    if ((DATA->Laps > 32500) | (DATA->Laps < -32500))
    {
        DATA->Laps = 0;
        DATA->Aim  = DATA->Angle_now;
    }

    DATA->ONLINE_JUDGE_TIME = 15;

    DATA->Angle_Infinite = (int32_t) ((DATA->Laps << 13)+ DATA->Angle_now);

}

/************************************************************万能分隔符**************************************************************
 *	@performance:	    //电机清空函数
 *	@parameter:		    //
 *	@time:				//23-04-13 19:23
 *	@ReadMe:			//
 *  @LastUpDataTime:    //2023-05-07 17:06    bestrui
 *  @UpData：           //不太好描述
 ************************************************************万能分隔符**************************************************************/
void HEAD_MOTOR_CLEAR(DJI_MOTOR_Typedef* MOTOR , uint8_t mode)
{
    MOTOR->PID_P.Iout  = 0.0f;
    MOTOR->PID_S.Iout  = 0.0f;
    MOTOR->DATA.Aim    = (float)MOTOR->DATA.Angle_Infinite;
    if (mode)       MOTOR->DATA.Laps = 0;
}

/************************************************************万能分隔符**************************************************************
 *	@performance:	    //2006电机堵转检测函数
 *	@parameter:		    //
 *	@time:				//23-04-13 20:31
 *	@ReadMe:			//
 *  @LastUpDataTime:    //2023-04-14 15:34    bestrui
 *  @UpData：           //给电机结构体增加堵转时间
 ************************************************************万能分隔符**************************************************************/
void HEAD_MOTOR2006_STUCK(DJI_MOTOR_Typedef* MOTOR , float ERROR_ANGLE , float ERROR_SPEED , uint16_t ERROR_TIME)
{
    //一定误差  速度小于某个值初步判断电机卡住，接下来去判断卡住的时间
    if (MATH_ABS_float(MOTOR->PID_P.Err) > ERROR_ANGLE && MATH_ABS_float(MOTOR->DATA.Speed_now) < ERROR_SPEED)
    {
        MOTOR->DATA.Stuck_Time++;
        if (MOTOR->DATA.Stuck_Time > ERROR_TIME)
        {
            HEAD_MOTOR_CLEAR(MOTOR, 0);
            MOTOR->DATA.Stuck_Time = 0;
            MOTOR->DATA.Stuck_Flag[0]++;
        }
    }
    else
    {
        MOTOR->DATA.Stuck_Time = 0;
    }
    MOTOR->DATA.Aim_last = MOTOR->DATA.Aim;

}

/************************************************************万能分隔符**************************************************************
 *	@performance:	    //3508堵转检测
 *	@parameter:		    //
 *	@time:				//23-05-25 18:14
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
void HEAD_MOTOR3508_STUCK(DJI_MOTOR_Typedef* MOTOR , float ERROR_CURRENT , float ERROR_SPEED)
{
    if (MATH_ABS_float(MOTOR->DATA.Speed_now) < ERROR_SPEED)
    {
        if (MATH_ABS_float(MOTOR->DATA.current) > ERROR_CURRENT)
        {
            HEAD_MOTOR_CLEAR(MOTOR, 0);
            MOTOR->DATA.Stuck_Flag[0]++;
        }
    }
}

void DJI_Current_Ctrl(hcan_t* hcan, uint16_t stdid, int16_t num1, int16_t num2, int16_t num3, int16_t num4)
{
    uint8_t Data[8];

    Data[0] = ((num1) >> 8);
    Data[1] = (num1);
    Data[2] = ((num2) >> 8);
    Data[3] = (num2);
    Data[4] = ((num3) >> 8);
    Data[5] = (num3);
    Data[6] = ((num4) >> 8);
    Data[7] = (num4);

    canx_send_data(hcan, stdid, Data,8);
}

/**
 * @brief 根据电机编码器值计算运转圈数
 *
 * @param motor 电机结构体指针
 */
void MotorRoundResolve(DJI_MOTOR_Typedef *motor)
{
    if (motor->DATA.Angle_now > 8091 && motor->DATA.Angle_last < 100) {
        motor->DATA.round-=1;
    }
    if (motor->DATA.Angle_now < 100 && motor->DATA.Angle_last > 8091) {
        motor->DATA.round+=1;
    }
    motor->DATA.conEncode=motor->DATA.round*8192+motor->DATA.Angle_now;
}
/*void MotorRoundResolve(DJI_MOTOR_Typedef *motor)
{
    #define OVERFLOW_HIGH_THRESH  7992  // 8192-200
    #define OVERFLOW_LOW_THRESH   200   // 0+200
    #define ENCODER_MAX           8192  // 编码器单圈最大值

    if(motor->DATA.Angle_now == motor->DATA.Angle_last)
    {
        motor->DATA.conEncode = motor->DATA.round * ENCODER_MAX + motor->DATA.Angle_now;
        return;
    }
    if (motor->DATA.Angle_now > OVERFLOW_HIGH_THRESH && motor->DATA.Angle_last < OVERFLOW_LOW_THRESH)
    {
        motor->DATA.round -= 1;
    }
    if (motor->DATA.Angle_now < OVERFLOW_LOW_THRESH && motor->DATA.Angle_last > OVERFLOW_HIGH_THRESH)
    {
        motor->DATA.round += 1;
    }
    motor->DATA.Angle_last = motor->DATA.Angle_now;
    motor->DATA.conEncode = motor->DATA.round * ENCODER_MAX + motor->DATA.Angle_now;
}*/

ALL_POWER_RX All_Power;
void CAN_POWER_Rx(Power_Typedef* Power, uint8_t *rx_data)
{
    int16_t raw_shunt = (int16_t)((rx_data[0] << 8) | rx_data[1]);
    int16_t raw_bus = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    int16_t raw_curr = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    int16_t raw_pwr = (int16_t)((rx_data[6] << 8) | rx_data[7]);

    Power->shunt_volt = (float)raw_shunt / 1000.0f;
    Power->bus_volt   = (float)raw_bus   / 1000.0f;
    Power->current    = (float)raw_curr  / 1000.0f;
    Power->power      = (float)raw_pwr   / 100.0f;
}

/**
************************************************************************
* @brief:      	DM_FBdata: 获取达妙电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @param[in]:   data_len: 数据长度
* @retval:     	void
* @details:    	从接收到的数据中提取达妙电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩相关温度参数、寄存器数据等
************************************************************************
**/
void DM_FBdata(DM_MOTOR_Typdef *motor, uint8_t *rx_data)
{
    //返回的数据有8个字节
    motor->DM_DATA.id = (rx_data[0])&0x0F;
    motor->DM_DATA.state = (rx_data[0])>>4;
    motor->DM_DATA.p_int=(rx_data[1]<<8)|rx_data[2];
    motor->DM_DATA.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
    motor->DM_DATA.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
    motor->DM_DATA.pos = uint_to_float(motor->DM_DATA.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
    motor->DM_DATA.vel = uint_to_float(motor->DM_DATA.v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
    motor->DM_DATA.tor = uint_to_float(motor->DM_DATA.t_int, T_MIN, T_MAX, 12);  // (-10.0,10.0)
    motor->DM_DATA.Tmos = (float)(rx_data[6]);
    motor->DM_DATA.Tcoil = (float)(rx_data[7]);
    motor->DM_DATA.ONLINE_JUDGE_TIME = 15;

}

int16_t angleError = 0;
#define df_now 0
#define df_last 1
#define df_llast 2
void DM_RXdata(DM_MOTOR_Typdef  *motor, uint8_t *rx_data) //一拖四模式下
{

	motor->para.pos[df_last] = motor->para.pos[df_now];
	motor->para.pos[df_now]  = ((rx_data[0] << 8)|(rx_data[1]));
	int16_t spd_int16= ((rx_data[2] << 8)|(rx_data[3]));
	int16_t cur_int16 = (rx_data[4] << 8)|(rx_data[5]);
	angleError =	motor->para.pos[df_now] -motor->para.initialAngle;
	if(angleError > 4096){
			angleError -= 8192;
	}
	else if (angleError < -4096){
			angleError += 8192;
	}
	motor->para.ralativeAngle = angleError * 0.044f;
	if(( motor->para.pos[df_now] - 	motor->para.pos[df_last])<-4096)
	{
		motor->para.round++;
	}
	else if(( motor->para.pos[df_now] - 	motor->para.pos[df_last])>4096)
	{
	  motor->para.round--;
	}

	/*圈数清零保证不会疯转*/
	if((motor->para.round > 32000) | (motor->para.round < -32000))
	{
    motor->para.round = 0;
		motor->para.aim = motor->para.pos[df_now] ;
	}
	motor->para.vel[df_last] = motor->para.vel[df_now];
	motor->para.vel[df_now] =  spd_int16/100;

	motor->para.vel[df_now]= OneFilter1(motor->para.vel[df_now],motor->para.vel[df_last] , 500);
	motor->para.cur =  ((float)cur_int16);//(16384.0f/20.0f);
	motor->para.Tcoil = (float)(rx_data[6]);
	motor->para.Tmos = (float)(rx_data[7]);
  motor->para.reality = (int32_t)(( motor->para.round * 8192)+(float)(motor->para.pos[df_now]));///8192.0f*360.0f;
	 motor->para.ONLINE_JUDGE_TIME=50;
}

//电机模式选择
void motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, DMMotor_Mode_e what)
{
  uint8_t data[8];
  uint16_t id = motor_id + mode_id;

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = what;

  canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
  uint8_t data[8];
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
  vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
  kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
  kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
  tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
  data[7] = tor_tmp;

  canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, DM_MOTOR_PV_Typdef *PVS)
{
  uint16_t id;
  uint8_t *pbuf, *vbuf;
  uint8_t data[8];

  id = motor_id + POS_MODE;
  pbuf=(uint8_t*)&PVS->POS;
  vbuf=(uint8_t*)&PVS->SPE;

  data[0] = *pbuf;
  data[1] = *(pbuf+1);
  data[2] = *(pbuf+2);
  data[3] = *(pbuf+3);

  data[4] = *vbuf;
  data[5] = *(vbuf+1);
  data[6] = *(vbuf+2);
  data[7] = *(vbuf+3);

  canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void speed_ctrl(hcan_t* hcan,uint16_t motor_id, DM_MOTOR_V_Typdef *vel)
{
  uint16_t id;
  uint8_t *vbuf;
  uint8_t data[4];

  id = motor_id + SPEED_MODE;
  vbuf=(uint8_t*)&vel->SPE;

  data[0] = *vbuf;
  data[1] = *(vbuf+1);
  data[2] = *(vbuf+2);
  data[3] = *(vbuf+3);

  canx_send_data(hcan, id, data, 4);
}
void DM_current_set(hcan_t* hcan, uint16_t id, float m1_cur_set, float m2_cur_set, float m3_cur_set, float m4_cur_set)
{
	uint8_t data[8];

	int16_t m1_cur_tmp = m1_cur_set;
	int16_t m2_cur_tmp = m2_cur_set;
	int16_t m3_cur_tmp = m3_cur_set;
	int16_t m4_cur_tmp = m4_cur_set;

	data[0] = (m1_cur_tmp >> 8);
	data[1] =  m1_cur_tmp;

	data[2] = (m2_cur_tmp >> 8);
	data[3] =  m2_cur_tmp;

	data[4] = (m3_cur_tmp >> 8);
	data[5] =  m3_cur_tmp;

	data[6] = (m4_cur_tmp >> 8);
	data[7] =  m4_cur_tmp;

	canx_send_data(hcan, id, data, 8);
}



int16_t OneFilter1(int16_t last, int16_t now, float thresholdValue)
{
		//减小平滑滤波值会增大对于细小毛刺的过滤程度
		//增加尖峰滤波值会增大对于尖峰数值的响应程度
	const float sensitivlFilter = 0.8f;	//尖峰滤波值//小于1//一般为最大值的20%
	const float numbFilter = 0.8f;	//平滑滤波值//小于1

	if((last - now)>= thresholdValue){
			return (float)( now*sensitivlFilter + last*(1-sensitivlFilter) );
	}
	else{
			return (float)( now*numbFilter + last*(1-numbFilter) );
	}
}
/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //电机清空函数
 *	@parameter:		    //
 *	@time:				//23-04-13 19:23
 *	@ReadMe:			//
 *  @LastUpDataTime:    //2023-05-07 17:06    bestrui
 *  @UpData：           //不太好描述
 ************************************************************万能分隔符**************************************************************/


