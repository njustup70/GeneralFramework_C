/**
 * @file motor_dji.hpp
 * @author https://github.com/Huangney
 * @date 2025-9-7
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "bsp_can.h"
#include "pids.hpp"

#define ABS(x) ((x > 0) ? (x) : (-x))
#define Lim_ABS(x, y) \
if (x > y) 			\
{		   			\
	x = y; 			\
}					\
else if (x < -y) 	\
{					\
	x = -y;			\
}

// 电机反馈信息结构体变量定义
typedef struct
{
	int16_t speed_rpm;
	int16_t real_current;
	uint8_t temprature;			
	uint16_t angle;		 		// 范围为[0, 8191]
	uint16_t last_angle;
	uint16_t offset_angle;
	int32_t round_cnt;
	int32_t total_angle;
	uint32_t msg_cnt;
} moto_measure_t;

typedef enum
{
	Pos_Control,
	Speed_Control,
	None_Control
}MotorDJIMode;


/// @brief 电机类：C620 / C610
class MotorDJI
{
private:
	friend void _MotorDJI_DecodeMeasure(MotorDJI* motor_p, uint8_t *Data);
	// 是否启用电机控制，若为否，发送0电流指令或不发送指令
	bool enabled = false;
	/// @brief 电机是否在线（私有）
	bool _online_priv = false;
	int _online_cnt = 0; 
	/// @brief 电流限幅		
	uint16_t _current_limit = 8000;
	/// @brief 速度限幅 	(减速比前的RPM)
	uint16_t _speed_limit = 20000;
	/// @brief 爬坡率限制	(单位：current/s)
	uint32_t _sloperate = 600000;
	

	/// @brief 前10次接收数据的平均时间间隔(最小单位：0.1ms，uint16精度)，用于判断电机在线质量
	uint32_t _recv_tick = 0;
	uint32_t _recv_sum_interval = 0;
	uint16_t _recv_interval[10] = {0};
	uint8_t _recv_last_index = 0;
	bool _recv_looped = false;
	/// @brief 根据前十次间隔计算的到的频率 (单位Hz)
	float _recv_freq = 0.0f;

	/// @brief 电机转速低通滤波系数（转速的低通滤波会导致严重的延迟）
	float _read_rpm_lpf_rate = 1.0f; 
	/// @brief 电机电流低通滤波系数
	float _read_current_lpf_rate = 0.5f; 

	/** 	  方法		**/
	/// @brief 电机速度环控制 
	void _MotorDJI_SpeedLoop();
	/// @brief 电机位置环控制
	void _MotorDJI_PosLoop();
	/// @brief 获取电机所在的CAN段，用于发送
	uint8_t _GetCanSeg(uint8_t motor_id);
	
public:
	MotorDJI(){};

	/** 	  方法		**/
	void Init(CAN_HandleTypeDef *hcan, uint8_t motorESC_id, MotorDJIMode djimode, bool fastInit = true);
	void SwitchMode(MotorDJIMode new_mode);
	void SetSpeed(float rpm, float redu_ratio = 19.0f);			// 3508的默认减速比（用2006的时候记得改！）
	void SetPos(float pos);
	void Neutral();
	void Disable();
	void Enable();
	bool IsEnabled();											// 返回电机控制是否启用
	int16_t Control();

	/** 	静态方法	**/
	static void ControlAllMotors();
		


	/** 	信息流变量	**/
	/// @brief 电机反馈信息结构体
	moto_measure_t measure;	

	/// @brief 电机是否在线（公开但只读）
	const bool& online = _online_priv;	

	/// @brief 电机的CAN实例
	BspCan_Instance bspcan_inst;

	/// @brief 当前电机所在的CAN段，用于发送电流 注：0-3分别对应CAN1的1-4号电机，5-8号电机，CAN2的1-4号电机，5-8号电机
	uint8_t at_can_seg = 0;
	
	/** 	控制用变量	**/
	Pids speed_pid; 					// 速度环PID
	Pids position_pid; 					// 位置环PID
	float targ_position = 0;			// 目标位置
	float targ_speed = 0;		    	// 目标速度
	float targ_current = 0;				// 目标电流

	/**		属性类变量	**/
	MotorDJIMode mode = None_Control;	// 电机当前控制模式
};

namespace MotorDJIConst
{
	const static float redu_M3508 = 19.0f;	// M3508电机的减速比
	const static float redu_M2006 = 36.0f;	// M2006电机的减速比
}

typedef MotorDJI MotorC610;		// 本库对C620和C610的支持是通用的
typedef MotorDJI MotorC620;		



#ifdef __cplusplus
}
#endif
