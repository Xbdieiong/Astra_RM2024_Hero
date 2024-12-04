/**
  ******************************************************************************
  * File Name          : BSP_IMU.h
  * Description        : �弶IMU֧�ְ�ͷ�ļ�
	* Hardware           : DJI_C
  ******************************************************************************
  * @attention IMU		 : BMI088
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_IMU_H
#define __BSP_IMU_H
#ifdef __cplusplus
	extern "C" {
#endif

#include "main.h"
#include "spi.h"
#include "tim.h"
		
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"	
		
typedef struct{
	float Angle_X;//ŷ����
	float Angle_Y;
	float Angle_Z;

	float Palst_X;//���ٶ�
	float Palst_Y;
	float Palst_Z;
	
	float Accel_X;//���ٶ�
	float Accel_Y;
	float Accel_Z;	
	
	float Temp;		//�¶�
} IMU_t;	
		
		
void BSP_IMU_Init(void);
IMU_t *BSP_IMU_Point(void);
		
#define PART_ACCEL  	0
#define PART_GYRO 		1		
		
	
#ifdef __cplusplus
	}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/**********************************END OF FILE*********************************/
