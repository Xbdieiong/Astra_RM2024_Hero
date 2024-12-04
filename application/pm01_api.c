/*
 ***********************************************************************************
 *
 * @file     pm01_api.c
 * @brief    PM01�������ݿ��������Ƴ���API
 * @author   hepeng(1130001324@qq.com)
 *
 * Copyright (c) 2017-2021, �������������Ƽ����޹�˾ All rights reserved. 
 * 
 ***********************************************************************************
 * History:
 * 2021-07-08    hepeng   ����v1.0
 ***********************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "pm01_api.h"
#include "CAN_receive.h"
static  power_measure_t power_data[4];

volatile pm01_od_t pm01_od;

volatile uint16_t  pm01_access_id;    /* ���ڷ��ʵı�ʶ��     */
volatile uint16_t  pm01_response_flg; /* ��������Ӧ�ɹ���־λ */

//���г������ݻ�ȡ��ѹ
void get_power_measure(power_measure_t *ptr,uint8_t *data1)
{
 uint16_t data2[4]; 
 data2[0] = data1[1]<<8 | data1[0];
 data2[1] = data1[3]<<8 | data1[2];
 data2[2] = data1[5]<<8 | data1[4];
 data2[3] = data1[7]<<8 | data1[6];
 
 ptr->InputVot = (float)data2[0]/100.f;
 ptr->CapVot = (float)data2[1]/100.f;
 ptr->Input_Current = (float)data2[2]/100.f;
 ptr->Target_Power = (float)data2[3]/100.f;
}





/*ԭ500����ӿ��ư���� ���������ù��ʵķ��͵�ַ--0x210
  * @brief          ���������
  * @param[in]      new_cmd   0x00:  ͣ��
															0x01: ���У�����������ؿ��أ�ֻ���������ݳ�磩
															0x02: ���У���������ؿ��أ���������ʹ�ø�ָ�
	                  save_flg: 0x00: ��������EEPROM  0x01: ������EEPROM
  * @retval         none
  */
void pm01_cmd_send( uint16_t new_cmd, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];

	power_tx_message.StdId = 0x600;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	power_can_send_data[0] = (uint8_t)(new_cmd >> 8   );
	power_can_send_data[1] = (uint8_t)(new_cmd &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = (save_flg == 0x01);			

	HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);
	
}

/**
  * @brief          ���ù���
  * @param[in]      new_power���µĹ���ֵ
                    save_flg: 0x00: ��������EEPROM  0x01: ������EEPROM
  * @retval         none
  */
void pm01_power_set( uint16_t new_power, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];

	power_tx_message.StdId = 0x210;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	power_can_send_data[0] = (uint8_t)(new_power >> 8   );
	power_can_send_data[1] = (uint8_t)(new_power &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = (save_flg == 0x01);			

	HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);
	
}
/**
  * @brief          ���������ѹ
  * @param[in]      new_volt���µĵ�ѹֵ
                    save_flg: 0x00: ��������EEPROM  0x01: ������EEPROM
  * @retval         none
  */
void pm01_voltage_set( uint16_t new_voltage, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];


	power_tx_message.StdId = 0x602;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	power_can_send_data[0] = (uint8_t)(new_voltage >> 8   );
	power_can_send_data[1] = (uint8_t)(new_voltage &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = (save_flg == 0x01);			


	HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);
	
}
/**
  * @brief          ���������ѹ
  * @param[in]      new_volt���µĵ�ѹֵ
                    save_flg: 0x00: ��������EEPROM  0x01: ������EEPROM
  * @retval         none
  */
void pm01_current_set( uint16_t new_current, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];

	power_tx_message.StdId = 0x603;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	power_can_send_data[0] = (uint8_t)(new_current >> 8   );
	power_can_send_data[1] = (uint8_t)(new_current &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = (save_flg == 0x01);			

	HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);
	
}
/**
  * @brief          ��ѯ����PM01����������100HZƵ�ʵ���
  * @param[in]      none
  * @retval         none
  */
void pm01_access_poll(void)
{
	
	static uint8_t   m_state   = 0x00;   /* ״̬    */
	static uint16_t  m_can_id  = 0x600;  /* ��ʶ��  */
	static uint32_t  m_timeout = 0;      /* ��ʱ    */
	
	static uint16_t  i;
	
	CAN_TxHeaderTypeDef  power_tx_message;

	uint8_t              power_can_send_data[8];

	uint32_t             send_mail_box;

	switch( m_state )
	{
		case 0x00:  /* �������� */
		
		  m_timeout = 0;   /* �����ʱ��ʱ�� */		

		  m_state = 0x01;  /* �л����ȴ�״̬ */

			pm01_access_id = m_can_id; /* ��¼���η��ʵı�ʶ�� */
		
			power_tx_message.StdId = m_can_id;       /* ������Ӧ�ı�ʶ�� */
			power_tx_message.IDE   = CAN_ID_STD;     /* ��׼֡           */
			power_tx_message.RTR   = CAN_RTR_REMOTE; /* Զ��֡           */
			power_tx_message.DLC   = 0x00;
		
			HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);	
   
			break;
		case 0x01: /* �ȴ���Ӧ */
		
	
      if( pm01_response_flg == 0x01 )
			{
				
				pm01_response_flg = 0x00; /* ��λӦ���־λ */
				
				 /* ���ʳɹ�������������һ����ʶ�� */
				
				switch(++i % 8)
				{
					case 0: m_can_id = 0x600; break;
					case 1: m_can_id = 0x601; break;
					case 2: m_can_id = 0x602; break;
					case 3: m_can_id = 0x603; break;
					case 4: m_can_id = 0x610; break;
					//case 5: m_can_id = 0x611; break;
					case 6: m_can_id = 0x612; break;
					//case 7: m_can_id = 0x613; break;
				}
				
				m_state = 0x00;  /* ������һ�ַ��� */							
				
			}
			else
			{
				m_timeout++;
			}

		  /* ��ʱ50ms */
      if( m_timeout > 5 )
			{
				m_state = 0x00;  /* ���·��� */			
			}				
		
			break;
		  default:
			m_state = 0x00;
	
	}

}
/*
  * @brief          Ӧ������CAN�����ж��е���
  * @param[in]      none
  * @retval         none
  */
void pm01_response_handle(CAN_RxHeaderTypeDef  *can_rx_header, uint8_t *can_rx_data )
{

	uint16_t m_tmp;
	
	pm01_response_flg = ( pm01_access_id == can_rx_header->StdId );
	
	if( can_rx_header->RTR == CAN_RTR_REMOTE )return;
	
	switch( can_rx_header->StdId )
	{
		case 0x600:
		
      m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	
		
		  pm01_od.ccr = m_tmp;
		
			break;
		case 0x601:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.p_set = m_tmp;
		
			break;
		case 0x602:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.v_set = m_tmp;
					
			break;
		case 0x603:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.i_set = m_tmp;
					
			break;
		case 0x610:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.sta_code.all = m_tmp;
		
		  m_tmp = (uint16_t)can_rx_data[2] << 8 | can_rx_data[3];	

			pm01_od.err_code = m_tmp;
		
			break;
//		case 0x611:
//			
//		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

//			pm01_od.p_in = m_tmp;
//		
//		  m_tmp = (uint16_t)can_rx_data[2] << 8 | can_rx_data[3];	

//			pm01_od.v_in = m_tmp;	
//		
//		  m_tmp = (uint16_t)can_rx_data[4] << 8 | can_rx_data[5];	

//			pm01_od.i_in = m_tmp;
//		
//			break;
		case 0x612:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.p_out = m_tmp;
		
		  m_tmp = (uint16_t)can_rx_data[2] << 8 | can_rx_data[3];	

			pm01_od.v_out = m_tmp;	
		
		  m_tmp = (uint16_t)can_rx_data[4] << 8 | can_rx_data[5];	

			pm01_od.i_out = m_tmp;
					
			break;
//		case 0x613:
//			
//		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

//			pm01_od.temp = m_tmp;
//		
//		  m_tmp = (uint16_t)can_rx_data[2] << 8 | can_rx_data[3];	

//			pm01_od.total_time = m_tmp;	
//		
//		  m_tmp = (uint16_t)can_rx_data[4] << 8 | can_rx_data[5];	

//			pm01_od.run_time = m_tmp;
//					
//			break;
	}
	Capacitor_Voltage = pm01_od.v_out/100.0;

}
//���е��ݿ��ƴ���
/**
  * @brief          �������ù���
  * @param[in]      new_power���µĹ���ֵ
                    save_flg: 0x00: ��������EEPROM  0x01: ������EEPROM
  * @retval         none
  */
void super_cape_power_set ( uint16_t new_power)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];
	power_tx_message.StdId = 0x210;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x08;
	power_can_send_data[0] = new_power >> 8  ;
	power_can_send_data[1] = new_power;

	HAL_CAN_AddTxMessage(&hcan1, &power_tx_message, power_can_send_data, &send_mail_box);
}

/**
  * @brief          ���л�ȡ��ѹ
  * @param[in]      new_power���µĹ���ֵ
                    save_flg: 0x00: ��������EEPROM  0x01: ������EEPROM
  * @retval         none
  */
void super_cape_voltage_get(power_measure_t *ptr,uint8_t *data1)
{
	uint16_t data2[4]; 
	data2[0] = data1[1]<<8 | data1[0];
	data2[1] = data1[3]<<8 | data1[2];
	data2[2] = data1[5]<<8 | data1[4];
	data2[3] = data1[7]<<8 | data1[6];
	
	ptr->InputVot = (float)data2[0]/100.f;
	ptr->CapVot = (float)data2[1]/100.f;
	ptr->Input_Current = (float)data2[2]/100.f;
	ptr->Target_Power = (float)data2[3]/100.f;
	
}
//����ԭA����벿�ֶ���
/*���е���CANbus
//			case CAN_BUS1_CAPACITOR_FEEDBACK_MSG_ID://��������700
//			{
//				Capacitor_PowerProcess(&Cap_PowerData, msg);
//				//wwdg_flag |=0x0060;
//			}break;
			*/
/*��������700 can��
//void Set_Capacitor_Power(CAN_TypeDef *CANx,int16_t can1_capacitor_power)
//{
//    CanTxMsg tx_message;    
//    tx_message.StdId = 0x210;
//    tx_message.IDE = CAN_Id_Standard;
//    tx_message.RTR = CAN_RTR_Data;
//    tx_message.DLC = 0x08;

//    tx_message.Data[0] = (unsigned char)(can1_capacitor_power>> 8);
//    tx_message.Data[2] = (unsigned char) can1_capacitor_power;
//    tx_message.Data[1] = 0x00;
//    tx_message.Data[3] = 0x00;
//    tx_message.Data[4] = 0x00;
//    tx_message.Data[5] = 0x00;
//    tx_message.Data[6] = 0x00;
//    tx_message.Data[7] = 0x00;
//    CAN_Transmit(CANx,&tx_message);
//}
*/
//����powerin��powerout
/*
//void Capacitor_PowerINProcess(volatile PowerINData *p, CanRxMsg * msg)
//{
//	p->input_power= (float)((msg->Data[0]<<8)|msg->Data[1])/100.0f;
//	p->input_voltage= (float)((msg->Data[2]<<8)|msg->Data[3])/100.0f;
//	p->input_current = (float)((msg->Data[4]<<8)|msg->Data[5])/100.0f;
//}

//void Capacitor_PowerOUTProcess(volatile PowerOUTData *p, CanRxMsg * msg)
//{
//	p->output_power= (float)((msg->Data[0]<<8)|msg->Data[1]);///100.0f;
//	p->output_voltage= (float)((msg->Data[2]<<8)|msg->Data[3]);///100.0f;
//	p->output_current = (float)((msg->Data[4]<<8)|msg->Data[5]);///100.0f;
//	Capacitor_Voltage = p->output_voltage;
//}
*/

