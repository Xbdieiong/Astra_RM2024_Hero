/**
  ******************************************************************************
  * File Name          : BSP_IMU.cpp
  * Description        : 板级IMU支持包
	* Hardware           : DJI_C
  ******************************************************************************
  * @attention 需要在main函数中进行初始化 依赖LPF PID
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "BSP_IMU.h"
#include "BSP_IMU_REG.h"
#include "DebuggerThread.h"

#include <math.h>
#include "LPF.h"
#include "PID.h"

#define SampleFreq	1000.0f			// sample frequency in Hz
#define TwoKp	(2.0f * 0.5f)			// 2 * proportional gain
#define TwoKi	(2.0f * 0.0f)			// 2 * integral gain

extern "C" {}
static IMU_t IMU;
unsigned int BMI088_ErrorCode = 0;
/* Functions -----------------------------------------------------------------*/
/**
  * @brief 读写BMI088的数据
  * @retval None
  */
static unsigned char BMI088_WriteRead(unsigned char Data){
	unsigned char Ret;
	HAL_SPI_TransmitReceive(&hspi1,&Data,&Ret,1,100);
	return Ret;
}
/**
  * @brief 写BMI088的单字节寄存器
	*	@argument Part:(0:Accel 1:Gyro)
  * @retval None
  */
static void BMI088_WriteSingleReg(unsigned char Part,unsigned char Addr,unsigned char Data){
	HAL_GPIO_WritePin(Part?CS1_GYRO_GPIO_Port:CS1_ACCEL_GPIO_Port,Part?CS1_GYRO_Pin:CS1_ACCEL_Pin,GPIO_PIN_RESET);
	BMI088_WriteRead(Addr);
	BMI088_WriteRead(Data);
	HAL_GPIO_WritePin(Part?CS1_GYRO_GPIO_Port:CS1_ACCEL_GPIO_Port,Part?CS1_GYRO_Pin:CS1_ACCEL_Pin,GPIO_PIN_SET);
}
/**
  * @brief 读BMI088的单字节寄存器
	*	@argument Part:(0:Accel 1:Gyro)
  * @retval None
  */
static void BMI088_ReadSingleReg(unsigned char Part,unsigned char Addr,unsigned char *Data){
	HAL_GPIO_WritePin(Part?CS1_GYRO_GPIO_Port:CS1_ACCEL_GPIO_Port,Part?CS1_GYRO_Pin:CS1_ACCEL_Pin,GPIO_PIN_RESET);
	BMI088_WriteRead(Addr | 0x80);
	if(!Part){
		BMI088_WriteRead(0x55);
	}
	*Data = BMI088_WriteRead(0x55);
	HAL_GPIO_WritePin(Part?CS1_GYRO_GPIO_Port:CS1_ACCEL_GPIO_Port,Part?CS1_GYRO_Pin:CS1_ACCEL_Pin,GPIO_PIN_SET);
}
/**
  * @brief 读BMI088的多字节寄存器
	*	@argument Part:(0:Accel 1:Gyro)
  * @retval None
  */
static void BMI088_ReadMultiReg(unsigned char Part,unsigned char Addr,unsigned char *Data,unsigned int Len){
	HAL_GPIO_WritePin(Part?CS1_GYRO_GPIO_Port:CS1_ACCEL_GPIO_Port,Part?CS1_GYRO_Pin:CS1_ACCEL_Pin,GPIO_PIN_RESET);
	BMI088_WriteRead(Addr | 0x80);
	if(!Part){
		BMI088_WriteRead(0x55);
	}
	while(Len){
		*Data = BMI088_WriteRead(0x55);
		Data++;
		Len--;
	}
	HAL_GPIO_WritePin(Part?CS1_GYRO_GPIO_Port:CS1_ACCEL_GPIO_Port,Part?CS1_GYRO_Pin:CS1_ACCEL_Pin,GPIO_PIN_SET);
}
/**
  * @brief 初始化BMI088
  * @retval None
  */
static void BMI088_Init(void){
	unsigned char Reg = 0;
	
	//check commiunication is normal
	BMI088_ReadSingleReg(PART_ACCEL,BMI088_ACC_CHIP_ID,&Reg);
	osDelay(1);
	BMI088_ReadSingleReg(PART_ACCEL,BMI088_ACC_CHIP_ID,&Reg);	
	osDelay(1);	
	
	//reset accel sensor and wait for > 50ms
	BMI088_WriteSingleReg(PART_ACCEL,BMI088_ACC_SOFTRESET,BMI088_ACC_SOFTRESET_VALUE);
	osDelay(80);
	
	//check commiunication is normal
	BMI088_ReadSingleReg(PART_ACCEL,BMI088_ACC_CHIP_ID,&Reg);
	osDelay(1);
	BMI088_ReadSingleReg(PART_ACCEL,BMI088_ACC_CHIP_ID,&Reg);	
	osDelay(1);	

	if(Reg != BMI088_ACC_CHIP_ID_VALUE){
		BMI088_ErrorCode |= 1;
	}
	
	BMI088_WriteSingleReg(PART_ACCEL,BMI088_ACC_PWR_CTRL,BMI088_ACC_ENABLE_ACC_ON);
	osDelay(1);
	BMI088_WriteSingleReg(PART_ACCEL,BMI088_ACC_PWR_CONF,BMI088_ACC_PWR_ACTIVE_MODE);	
	osDelay(1);
	BMI088_WriteSingleReg(PART_ACCEL,BMI088_ACC_CONF,BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set);	
	osDelay(1);
	BMI088_WriteSingleReg(PART_ACCEL,BMI088_ACC_RANGE,BMI088_ACC_RANGE_3G);	
	osDelay(1);
	BMI088_WriteSingleReg(PART_ACCEL,BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW);	
	osDelay(1);
	BMI088_WriteSingleReg(PART_ACCEL,BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT);	
	osDelay(1);	
	
	//check commiunication is normal
	BMI088_ReadSingleReg(PART_GYRO,BMI088_GYRO_CHIP_ID,&Reg);
	osDelay(1);
	BMI088_ReadSingleReg(PART_GYRO,BMI088_GYRO_CHIP_ID,&Reg);	
	osDelay(1);	
	
	//reset the gyro sensor and wait for > 50ms
	BMI088_WriteSingleReg(PART_GYRO,BMI088_GYRO_SOFTRESET,BMI088_GYRO_SOFTRESET_VALUE);
	osDelay(80);
	
	//check commiunication is normal
	BMI088_ReadSingleReg(PART_GYRO,BMI088_GYRO_CHIP_ID,&Reg);
	osDelay(1);
	BMI088_ReadSingleReg(PART_GYRO,BMI088_GYRO_CHIP_ID,&Reg);	
	osDelay(1);
	
	if(Reg != BMI088_GYRO_CHIP_ID_VALUE){
		BMI088_ErrorCode |= 2;
	}
	
	BMI088_WriteSingleReg(PART_GYRO,BMI088_GYRO_RANGE,BMI088_GYRO_2000);
	osDelay(1);
	BMI088_WriteSingleReg(PART_GYRO,BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set);
	osDelay(1);	
	BMI088_WriteSingleReg(PART_GYRO,BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE);
	osDelay(1);
	BMI088_WriteSingleReg(PART_GYRO,BMI088_GYRO_CTRL, BMI088_DRDY_ON);
	osDelay(1);
	BMI088_WriteSingleReg(PART_GYRO,BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW);
	osDelay(1);
	BMI088_WriteSingleReg(PART_GYRO,BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3);
	osDelay(1);
}
/**
  * @brief 读取BMI088原始数据 单位: m/(s^2),  rad/s,  ℃
  * @retval None
  */
static void BMI088_Read(float Gyro[3],float Accel[3],float *Temperate){
	uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
	int16_t bmi088_raw_temp;

	BMI088_ReadMultiReg(PART_ACCEL,BMI088_ACCEL_XOUT_L, buf, 6);
	bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
	Accel[0] = bmi088_raw_temp * 0.0008974358974f;
	bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
	Accel[1] = bmi088_raw_temp * 0.0008974358974f;
	bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
	Accel[2] = bmi088_raw_temp * 0.0008974358974f;

	BMI088_ReadMultiReg(PART_GYRO,BMI088_GYRO_CHIP_ID, buf, 8);
	bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
	Gyro[0] = bmi088_raw_temp * 0.00106526443603169529841533860381f;
	bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
	Gyro[1] = bmi088_raw_temp * 0.00106526443603169529841533860381f;
	bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
	Gyro[2] = bmi088_raw_temp * 0.00106526443603169529841533860381f;

	BMI088_ReadMultiReg(PART_ACCEL,BMI088_TEMP_M, buf, 2);
	bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
	if (bmi088_raw_temp > 1023){
		bmi088_raw_temp -= 2048;
	}
	*Temperate = bmi088_raw_temp * 0.125f + 23.0f;
}
/**
  * @brief Fast inverse square-root
  * @retval None
  */
static float InvSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
/**
  * @brief IMU algorithm
  * @retval None
  */
static void AHRS_Calculate(float Gyro[3],float Accel[3],float Angle[3]){
	static float q[4] = {1.0f,0.0f,0.0f,0.0f};
	static float IntegralFBx = 0.0f;
	static float IntegralFBy = 0.0f;
	static float IntegralFBz = 0.0f;//integral error terms scaled by Ki
	float RecipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float gx = Gyro[0];
	float gy = Gyro[1];
	float gz = Gyro[2]; 
	float ax = Accel[0];
	float ay = Accel[1];
	float az = Accel[2];
	//Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		//Normalise accelerometer measurement
		RecipNorm = InvSqrt(ax * ax + ay * ay + az * az);
		ax *= RecipNorm;
		ay *= RecipNorm;
		az *= RecipNorm;        

		//Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		//Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		//Compute and apply integral feedback if enabled
		if(TwoKi > 0.0f) {
			IntegralFBx += TwoKi * halfex * (1.0f / SampleFreq);	// integral error scaled by Ki
			IntegralFBy += TwoKi * halfey * (1.0f / SampleFreq);
			IntegralFBz += TwoKi * halfez * (1.0f / SampleFreq);
			gx += IntegralFBx;	//apply integral feedback
			gy += IntegralFBy;
			gz += IntegralFBz;
		}
		else {
			//prevent integral windup
			IntegralFBx = 0.0f;	
			IntegralFBy = 0.0f;
			IntegralFBz = 0.0f;
		}

		//Apply proportional feedback
		gx += TwoKp * halfex;
		gy += TwoKp * halfey;
		gz += TwoKp * halfez;
	}
	
	//Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / SampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / SampleFreq));
	gz *= (0.5f * (1.0f / SampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	//Normalise quaternion
	RecipNorm = InvSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= RecipNorm;
	q[1] *= RecipNorm;
	q[2] *= RecipNorm;
	q[3] *= RecipNorm;
	
	//Get angle
	Angle[0] = 180.0f / pi * asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
	Angle[1] = 180.0f / pi * atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
	Angle[2] = 180.0f / pi * atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]),2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
}
/**
  * @brief 	IMU线程函数
  * @retval None
  */
float Gyro[3],Accel[3],Temp;
float Angle[3];
void IMU_Task(void *argument){
	LPF_t LPF_Gyro[3];
	PID_t PID_Temp;
	unsigned char FirstHeatFlag = 1;
	float Bias[3],Count = 0;
	PID_Temp.Init(3500,0.2,0,1,9999,9999);
	do{
		BMI088_ErrorCode = 0;
		BMI088_Init();
		osDelay(100);
	}while(BMI088_ErrorCode);
	for(int i=0;i<3;i++){
		LPF_Gyro[i].Init(1,30);
	}
	HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
	while(1){
		
		BMI088_Read(Gyro,Accel,&Temp);
		
		Gyro[0] -= 0.00319833;
		Gyro[1] -= -0.000920025;		
		Gyro[2] -= -0.00337662;
		
		for(int i=0;i<3;i++){
			Gyro[i] = LPF_Gyro[i].Generate(Gyro[i]);
		}
		
//		Bias[0] += Gyro[0];
//		Bias[1] += Gyro[1];		
//		Bias[2] += Gyro[2];		
//		Count ++;
//		
//		Debugger[0] = Bias[0]/Count;
//		Debugger[1] = Bias[1]/Count;		
//		Debugger[2] = Bias[2]/Count;		
//		
		
		AHRS_Calculate(Gyro,Accel,Angle);
		IMU.Palst_X = Gyro[0];
		IMU.Palst_Y = Gyro[1];		
		IMU.Palst_Z = Gyro[2];
		
		IMU.Accel_X = Accel[0];
		IMU.Accel_Y = Accel[1];		
		IMU.Accel_Z = Accel[2];
		
		IMU.Angle_X = Angle[0];
		IMU.Angle_Y = Angle[1];		
		IMU.Angle_Z = Angle[2];
		IMU.Temp = Temp;
		
		Debugger[0] = Angle[0];
		Debugger[1] = Angle[1];
		Debugger[2] = Angle[2];
		Debugger[3] = Temp;
		
		if(FirstHeatFlag){
			if(Temp < 44.5f){
				__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,9999);
			}else{
				FirstHeatFlag = 0;
			}		
		}else{
			PID_Temp.Generate(Temp,45.0);
			__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,LimitBoth(1500+PID_Temp.GetOutput(),9999,0));
		}
		osDelay(1);
	}
}
/**
  * @brief 	IMU初始化
	*	@argument None
  * @retval None
  */
void BSP_IMU_Init(void){
  const osThreadAttr_t IMU_Task_attributes = {"IMU_Task",0,0,0,0,512,(osPriority_t) osPriorityNormal};
	osThreadNew(IMU_Task, NULL, &IMU_Task_attributes);
}
/**
  * @brief 	取板载IMU指针
	*	@argument None
  * @retval None
  */
IMU_t *BSP_IMU_Point(void){
	return &IMU;
}
/**********************************END OF FILE*********************************/
