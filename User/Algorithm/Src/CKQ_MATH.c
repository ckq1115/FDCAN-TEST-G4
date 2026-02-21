//
// Created by CaoKangqi on 2026/1/25.
//
#include "CKQ_MATH.h"

#include "stm32g473xx.h"

/************************************************************ 万能分隔符 **************************************************************
 *	@performance:	    // int16_t 绝对值
 *	@parameter:		    // DATA：需要计算绝对值的 int16_t 类型数据
 *	@ReadMe:			//
 ************************************************************************************************************************************/
int16_t MATH_ABS_int16_t(int16_t DATA)
{
    return DATA>>15 == 0 ? DATA : (~DATA + 1);
}

int32_t MATH_ABS_int32_t(int32_t DATA)
{
    if (DATA < 0) return -DATA;
    if (DATA > 0) return DATA;
	return 0;
}

int64_t MATH_ABS_int64_t(int64_t DATA)
{
    if (DATA < 0) return -DATA;
    if (DATA > 0) return DATA;
    return 0;
}

/************************************************************ 万能分隔符 **************************************************************
 *	@performance:	    // float 绝对值
 *	@parameter:		    // DATA：需要计算绝对值的 float 类型数据
 *	@ReadMe:			//
 ************************************************************************************************************************************/
float MATH_ABS_float(float DATA)
{
    uint32_t RUI_V_TEMP = *(uint32_t*) &DATA;
    RUI_V_TEMP &= 0x7FFFFFFF; // 将符号位清零
    return *(float*) &RUI_V_TEMP;
}

/************************************************************ 万能分隔符 **************************************************************
 *	@performance:	    // float 限幅
 *	@parameter:		    // MAX：上限值；MIN：下限值；DATA：需要限幅的 float 类型数据
 *	@ReadMe:			//
 ************************************************************************************************************************************/
float MATH_Limit_float(float MAX , float MIN , float DATA)
{
    return (DATA > MAX) ? MAX : ((DATA < MIN) ? MIN : DATA);
}

/************************************************************ 万能分隔符 **************************************************************
 *	@performance:	    // int16 限幅
 *	@parameter:		    // MAX：上限值；MIN：下限值；DATA：需要限幅的 int16_t 类型数据
 *	@ReadMe:			//
 ************************************************************************************************************************************/
int16_t MATH_Limit_int16(int16_t MAX , int16_t MIN , int16_t DATA)
{
    return (DATA > MAX) ? MAX : ((DATA < MIN) ? MIN : DATA);
}

/************************************************************ 万能分隔符 **************************************************************
 *	@performance:	    // 置位/复位单个比特位
 *	@parameter:		    // byte：待操作字节的指针；position：要设置的位所在位置（0到7）；value：要设置的值（1=置1，0=置0）
 *	@ReadMe:			//
 ************************************************************************************************************************************/
void MATH_SETBIT(unsigned char* byte , int position , int value)
{
    unsigned char mask = 1 << position;  // 生成一个只有指定位置为1的掩码
    if (value)
    {
        *byte |= mask;  // 将指定位置设置为1
    }
    else
    {
        *byte &= ~mask;  // 将指定位置设置为0
    }
}

/************************************************************ 万能分隔符 **************************************************************
 *	@performance:	    // float 平方根倒数
 *	@parameter:		    // DATA：需要计算平方根倒数的 float 类型数据
 *	@ReadMe:			//
 ************************************************************************************************************************************/
float MATH_INV_SQRT_float(float DATA)
{
    float DATA_half = 0.5f * DATA;
    uint32_t i = *(uint32_t*) &DATA; // 将浮点数视为无符号整数
    i = 0x5f3759df - (i >> 1); // 运用魔数进行处理
    DATA = *(float*) &i; // 再将无符号整数转回浮点数
    DATA = DATA * (1.5f - DATA_half * DATA * DATA); // 进行牛顿迭代
    return DATA;
}

//float uint_to_float(int16_t x_int, float span, int16_t value)
//{
//	return x_int-value/2/value*span;
//}

float Hex_To_Float(uint32_t *Byte,int num)// 十六进制到浮点数
{
  return *((float*)Byte);
}

uint32_t FloatTohex(float HEX)// 浮点数到十六进制转换
{
  return *( uint32_t *)&HEX;
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
  /* Converts a float to an unsigned int, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  /* converts unsigned int to float, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

float get_vbus_input(uint16_t value)
{
	return ((float)value*3.3f/65535)*11.0f;
}

void convertAngleToIndex(float angle, float *index) {
    // 角度标准化到0-360度
    while (angle < 0) {
        angle += 360;
    }
    while (angle >= 360) {
        angle -= 360;
    }

    // 将角度转换为0-8191的索引
    *index = (angle / 360.0f * 8192.0f);
}

/**
 * @brief CORDIC Atan2 快速函数
 */
inline float CORDIC_Atan2_Fast(float y, float x) {
    // 1. 使用 INT32_MAX 避免溢出 (2147483647.0f)
    const float f_q31 = 2147483647.0f;
    int32_t arg_x = (int32_t)(x * f_q31);
    int32_t arg_y = (int32_t)(y * f_q31);

    /* 2. 配置寄存器 (直接操作避免HAL开销)
       FUNC = 2 (Phase)
       PRECISION = 6 (24 cycles)
       NARG = 2 (必须为2，因为需要输入X和Y)
       NRES = 1
    */
    CORDIC->CSR = (2 << CORDIC_CSR_FUNC_Pos) |
                  (6 << CORDIC_CSR_PRECISION_Pos) |
                  (1 << CORDIC_CSR_NARGS_Pos) |   // NARG=1 在寄存器位定义中代表 2个参数
                  (0 << CORDIC_CSR_NRES_Pos);    // NRES=0 在寄存器位定义中代表 1个结果

    // 3. 严格按照顺序写入：先 X 后 Y
    CORDIC->WDATA = arg_x;
    CORDIC->WDATA = arg_y; // 写入第二个参数触发计算

    // 4. 读取结果
    int32_t res = CORDIC->RDATA;

    // 5. 转换回浮点 (res / 2^31 * 180.0)
    // 直接乘以这个系数可以直接得到角度度数，省去后续的 RAD2DEG
    // 系数 = 180.0 / 2147483648.0
    return (float)res * 8.38190317e-8f;
}