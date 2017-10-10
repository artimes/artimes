#ifndef __GLOBAL_H
#define __GLOBAL_H


#include <stdio.h>
#include <stdlib.h>
#include "IQmathLib.h"
#if defined (STM32F030)
 #include "stm32f0xx_nucleo.h"
#endif

/***********�汾�Ŷ���*****************/
#define VERSION_NUM   			2415262020


/**************************************/

/************������ص�ȫ�ֱ����ͺ���**********************/
#define OFF_BEGIN   			100
#define OFF_LEN2N   			10
#define OFF_END   				(OFF_BEGIN+(1<<OFF_LEN2N))
#define I_BASE      			75														//������ֵ
#define OverCurrMIN  			-_IQ(0.8667)   								//����ֵ����65A       
#define OverCurrMAX  			_IQ(0.8667)  									//����ֵ����65A 

#define CURRENT_T1        0 														//T1����������־λ


typedef struct
{
	int16_t u;
	int16_t v;
	int16_t w;
}Channels;

typedef struct 
{
	_iq vref1;
	_iq vref2;
	_iq vref3;
	_iq vref4;
	_iq vref5;

	_iq x;
	_iq y;
	_iq z;

	unsigned char a;
	unsigned char b;
	unsigned char c;
	unsigned char d;
	unsigned char sector;

	unsigned int T0;
	_iq T1;
	_iq T2;
	unsigned int T;
	
	unsigned int taon;
	unsigned int tbon;
	unsigned int tcon;
	unsigned int cmp1;
	unsigned int cmp2;
	unsigned int cmp3;	      	  	 

}SVPWM;
typedef struct {  
	        _iq  pi_ref_reg;   	
				  _iq  pi_fdb_reg;   
				  _iq  e_reg;			
				  long loop_index;		
				  _iq  e_limit;		
				  long i_period;	
				  long d_period;	
				  _iq  Kp_reg;		
          _iq  Ki_reg;	
				  _iq  Kd_reg;		
				  _iq  Kf_reg;      
				  _iq  ui_out_max;		
				  _iq  ui_out_min;
				  _iq  pi_out_max;		
				  _iq  pi_out_min;	
				  _iq  up_reg;		
				  _iq  ui_reg;
				  _iq  pi_out_reg;   	
				  long ctrl_period;
				 } PIREG;	     
extern SVPWM SVM;
extern volatile int16_t ADC_Values[7];
extern volatile uint8_t Flag_offset_finished;
extern volatile _iq Isa,Isb,Isc,I_Alpha,I_Beta,Id,Iq,Iq_Set,I_phase_sum;
extern volatile _iq Isa_sh,Isb_sh,Isc_sh,I_phase_sum_sh;
extern volatile _iq Isa_old,Isb_old ,Isc_old, Isa_d,Isb_d,Isc_d;
extern volatile uint32_t flag_all_low300;

extern volatile int16_t Iq_ave;						//����ƽ��ֵ
extern int16_t	Iq_ave_ave;
extern int16_t Current_lpf10;
extern volatile int32_t Iq_ref;

extern Channels Current_Offset;

 void Current_Process(void);
extern void checkOverCurrent(void);
extern void Clark(void);
extern void Park(void);
extern void current_ave(void);
extern 	int16_t Pos_Compute(void);
/**************************************/

/************��ѹ��ص�ȫ�ֱ���**********************/
extern uint16_t Vdc_lpf10;
extern uint8_t  Battery_level;	
extern uint16_t Power_value;							//��������ֵ�ı���
extern uint16_t U_delta;									//���ӵ�ѹ����
extern uint8_t Bat_remain;


extern uint8_t Flag_OV_start_ok;									//������ѹ��־λ
extern uint8_t Flag_UV_start_ok;									//����Ƿѹ��־λ

extern uint16_t PWM_double;
extern uint16_t PWM_max;
extern uint16_t PWM_half;

extern volatile _iq Ud;
extern volatile _iq Uq;

extern uint8_t  Flag_over_E;
extern void Voltage_Process(void);

typedef struct
{	
	int	mHalfPeriod;		// ??: ??HISPCP??????PWM?????(Q0)  	
	_iq		mDuty1;				// ??: PWM 1&2 ??? (QG)       		
	_iq		mDuty2;				// ??: PWM 3&4 ??? (QG)       		
	_iq		mDuty3;				// ??: PWM 5&6 ??? (QG)       		       
}PWMGEN;
extern PWMGEN pwma;

/************ת����ص�ȫ�ֱ���**********************/

/*
#define TORQUE_K_3  				_IQ(1)     
#define TORQUE_K_2  				_IQ(0.75)     
#define TORQUE_K_1  				_IQ(0.5)     
*/
/*
#define TORQUE_K_3  				_IQ(2)     
#define TORQUE_K_2  				_IQ(1.5)     
#define TORQUE_K_1  				_IQ(1)
*/
//#define KT                  1												//���ص�����

//#define FLAG_TEST_HANDLE    1							//�Ƿ��ڲ⹦����ʹ���հ�

extern volatile _iq TORQUE_K_3;
extern volatile _iq TORQUE_K_2;
extern volatile _iq TORQUE_K_1;

extern uint8_t FlagA;
extern uint8_t Flag_EN_ANG;
extern uint8_t Flag_tor_sin_OK;
extern uint8_t Flag_tor_start;
extern int16_t Torque_AD_lpf;
extern int16_t Torque_AD_lpf_protect;
extern int16_t Torque_AD_lpf_comp;  		//�����ȽϵĴ������˲�ֵ
extern int16_t Torque_lpf10;
extern int32_t Torque_Set;
	
extern void touque_angle_verify(void);
extern void torque_adc_lpf(void);
extern void torque_limit(void);
extern void torque_control_Iq_Set(void);


/************ͨ����ص�ȫ�ֱ���**********************/
extern volatile uint32_t cnt_no_receive_data_panel,cnt_no_receive_data_bat;

extern void tx_s16(void);
extern void tx_data_to_panel(void);
extern void recond_values_for_computer(void);
extern void rx_tx_data_with_computer(void);

extern volatile uint8_t Flag_err_bat_com;


/*************�����й�ȫ�ֱ���***********************/
extern volatile uint8_t Flag_Pha_OC;
extern uint8_t Flag_warning;
extern uint8_t Flag_run_state;									//ָʾ����״̬
extern uint32_t cnt_time_out;
extern void Protection_Check(void);

extern uint8_t Flag_0toput;

/*************ת���й�ȫ�ֱ���***********************/
extern uint8_t  Dirspd;														  //���ת���־λ
extern uint16_t RPM_lpf;
extern uint16_t RPM_lpf_10;
extern uint16_t speed_ave;
extern volatile uint16_t bike_speed;								//�Թ���Ϊ��λ�ĳ���
extern volatile uint16_t bike_speed_10;							//������10��
extern volatile uint16_t Fre_hall;									//����Ƶ��
extern volatile uint16_t Fre_hall_ave;							//ƽ������Ƶ��
extern volatile uint16_t Fre_hall_10;
extern volatile uint16_t Fre_hall_10_ave;						//ƽ������Ƶ��

extern uint8_t ANG_PIN;
extern uint8_t speed_low_flag;							        //���ٱ�־λ
extern uint8_t Flagbat;

#define KRPM      _IQ(0.05)												  //RPM����ϵ����ֵΪ���ٱȵĵ���

/*************λ���й�ȫ�ֱ���***********************/
extern volatile uint8_t Hall_state;								//����״̬
extern volatile uint8_t Flag_hall_state_err;
extern volatile int32_t ang_cos, ang_sin;
extern volatile int32_t ang_cos_temp, ang_sin_temp;
extern void Get_HallPositon(void);
extern void Motor_speed_angle_calculate(void);
extern void Angle_adjust(uint16_t angle);

extern int16_t PositionAngle;

#define A360                4096 		  //1800h
#define A270                3072  		//1800h
#define A180                2048  		//1000h
#define A90                 1024  		//800h

#define A30                341 		//30   degree(4096 / 360 * 30)
#define A150               1707 		//150  degree(4096 / 360 * 150)
#define A210               2389 		//210  degree(4096 / 360 * 210)
#define A330               3755 		//330  degree(4096 / 360 * 330)

/*************�¶��й�ȫ�ֱ���***********************/
extern uint16_t temp_lpf;
extern uint16_t temp_num;
extern int32_t Power_value_Temp; 
extern uint8_t Flag_Temp_start;



extern void Temp_Process(void);													//�¶ȼ����˲�����


/**************�����йر�־λ************************/
extern volatile uint8_t Flag_main_cycle;	
extern uint8_t Flag_temp_cut;
extern uint8_t Flag_phase_off;													//���߶�·��־λ	
extern uint8_t Flag_COM_good;



extern uint8_t Flag_power_mode;
extern uint8_t Flag_brake;
extern uint8_t Flag_brake_com;
extern uint8_t Flag_brake_bike;
extern uint8_t Flag_light;
extern uint8_t Flag_LS;

extern uint8_t Flag_power_off;
extern uint8_t Flag_test_mode;
extern volatile uint8_t Flag_switch_off;

extern uint8_t  temp_motor_low;	
extern uint8_t Flag_bat_data_err;
extern uint8_t state_temp_limit_I;
extern void Light_PowerOff_process(void);

extern void MotorDrive(void);
extern int16_t average_2(int32_t *sum, int16_t array[], uint8_t length2N, int16_t *index, uint8_t *flag_av_finished, int16_t value);

typedef   int8_t   int8;
typedef   int16_t  int16;
typedef   int16_t  s16;
typedef   int32_t  int32;
typedef   int32_t  s32;

typedef uint8_t   uint8;
typedef uint8_t   u8;
typedef uint16_t  uint16;
typedef uint16_t  u16;
typedef uint32_t  uint32;
typedef uint32_t  u32;





typedef struct
{
	int32 u;
	int32 v;
	int32 w;
}Channels32Bit;


/***************************************��ʱ�������趨*******************************************/
//�趨ֵΪTIM1PWMƵ�ʣ�TIM3����Ƶ�ʣ�PWM�Ƚ������ʽ���Ϲܺ��¹���Ч��ƽ��
#define TIM1_FREQ                 16000					//ѡ��PWMƵ��      16K
#define TIM3_FREQ                 100000				//ѡ��TM3����Ƶ��  100K
#define TIM1_PWM_MODE             2             //ѡ��PWM�����ʽ  PWM2
#define OCP                       TIM_OCPolarity_High //ѡ�����ű�Ϊ����Ч
#define OCN												TIM_OCNPolarity_Low //ѡ�����ű�Ϊ����Ч

#define T1_DIVIDER                32             //��Ƶϵ��,��Ƶ��500Hz
#define T_MAIN_20MS               10             //���������е�20ms
#define T_MAIN_200MS              100            //���������е�200ms
#define T_MAIN_1S									500						 //���������е�1����
#define T_MAIN_2S                 1000           //���������е�2����
#define T_MAIN_3S                 1500           //���������е�2����
#define T_MAIN_4S                 2000           //���������е�4����
#define T_MAIN_5S                 2500           //���������е�5����
#define T_MAIN_10S                5000           //���������е�10����
#define T_MAIN_1M                 30000          //���������е�10����
#define T_MAIN_2M									60000          //���������е�2����
#define T_MAIN_5M									150000				 //���������е�5����
/*******************************************************************************************/

/***************************************����ͨ�Ų����趨*************************************/

#define UART1_BAUDRATE            1200
#define UART2_BAUDRATE            9600	



#define COM_WITH_PANEL            1
/*******************************************************************************************/


/*******************************��ѹ��ز����趨**********************************************/
#define BAT_LEV_0_1								231				//��������0�͵�ѹ1�ķֽ� 10%//12
#define BAT_LEV_1_2								243				//��������1�͵�ѹ2�ķֽ� 30%//9
#define BAT_LEV_2_3								252				//��������2�͵�ѹ3�ķֽ� 50%//5
#define BAT_LEV_3_4								257				//��������2�͵�ѹ3�ķֽ� 75%//12
#define BAT_LEV_4_5								269				//��������2�͵�ѹ3�ķֽ� 75%+
#define BAT_LEV_DELTA           0         //������ʾ�ͻ����

#define BAT_0								  2310				//��������0�͵�ѹ1�ķֽ� 0%//126
#define BAT_10								2436				//��������1�͵�ѹ2�ķֽ� 10%//84
#define BAT_30								2520				//��������2�͵�ѹ3�ķֽ� 30%//56
#define BAT_50								2576				//��������2�͵�ѹ3�ķֽ� 50%//119
#define BAT_75								2695				//��������2�͵�ѹ3�ķֽ� 75% 205
#define BAT_100								2900				//��������2�͵�ѹ3�ķֽ� 100%+
/**********************Ϊ�������Ŷ�������������������ų�ʼ��********************************************/
//�޸�ʱ��ע�⽫�漰����GPIO_GROUPʱ����GPIO��ʼ�������д򿪣�����ֻ�õ�GPIOA/GPIOB/GPIOF

//��Դ������Ŷ���,����Ч��PA12
#define   POWER_PIN       			GPIO_Pin_12 
#define   POWER_PIN_GROUP       GPIOA

//���������Ŷ��壬����Ч��PF1
#define   LIGHT_PIN       			GPIO_Pin_11 
#define   LIGHT_PIN_GROUP       GPIOA

//ɲ���������Ŷ��壬����Ч��PF0
#define   BRAKE_PIN             GPIO_Pin_6
#define   BRAKE_PIN_GROUP       GPIOF

//��ƶ�·���Ŷ��壬����Ч��PF0
#define   LightShort_PIN             GPIO_Pin_12
#define   LightShort_PIN_GROUP       GPIOA

#define   EXTI_PortSource_LS    EXTI_PortSourceGPIOA
#define   EXTI_PinSource_LS     EXTI_PinSource12
#define   EXTI_Line_LS					EXTI_Line12
#define   EXTI_IRQn_LS          EXTI4_15_IRQn
#define   EXTI_IRQHandler_LS		EXTI4_15_IRQHandler


//HU�������Ŷ���
#define   HU_PIN                GPIO_Pin_5
#define   HU_PIN_GROUP          GPIOB

#define   EXTI_PortSource_HU    EXTI_PortSourceGPIOB
#define   EXTI_PinSource_HU     EXTI_PinSource5
#define   EXTI_Line_HU					EXTI_Line5
#define   EXTI_IRQn_HU          EXTI4_15_IRQn
#define   EXTI_IRQHandler_HU		EXTI4_15_IRQHandler

#define   EXTI_PortSource_HV    EXTI_PortSourceGPIOB
#define   EXTI_PinSource_HV     EXTI_PinSource4
#define   EXTI_Line_HV					EXTI_Line4
#define   EXTI_IRQn_HV          EXTI4_15_IRQn
#define   EXTI_IRQHandler_HV		EXTI4_15_IRQHandler

#define   EXTI_PortSource_HW    EXTI_PortSourceGPIOB
#define   EXTI_PinSource_HW     EXTI_PinSource3
#define   EXTI_Line_HW					EXTI_Line3
#define   EXTI_IRQn_HW          EXTI2_3_IRQn
#define   EXTI_IRQHandler_HW		EXTI2_3_IRQHandler



//HV�������Ŷ���
#define   HV_PIN                GPIO_Pin_4
#define   HV_PIN_GROUP          GPIOB

//HW�������Ŷ���
#define   HW_PIN                GPIO_Pin_3
#define   HW_PIN_GROUP          GPIOB

//��λ�������Ŷ���
#define   ANGLE_PLUSE_PIN         GPIO_Pin_13
#define   ANGLE_PLUSE_PIN_GROUP   GPIOC

//�����ź��������Ŷ��壬����Ч
#define   OVER_CURRENT_PIN        GPIO_Pin_12
#define   OVER_CURRENT_PIN_GROUP  GPIOB



/************************PWM������Ŷ���*****************************************/
//��ʼ������������ȷ��GPIO���ʱ�Ӵ�
//PWM UP������Ŷ���   ��TIM1_CH3������
#define   UP_PIN       					 GPIO_Pin_10
#define   UP_PIN_GROUP  				 GPIOA
#define   UP_PIN_SOURCE  				 GPIO_PinSource10

//PWM UN������Ŷ���
#define   UN_PIN       					 GPIO_Pin_15
#define   UN_PIN_GROUP  				 GPIOB
#define   UN_PIN_SOURCE  				 GPIO_PinSource15

//PWM VP������Ŷ���   ��TIM1_CH2������
#define   VP_PIN       					 GPIO_Pin_9
#define   VP_PIN_GROUP  				 GPIOA
#define   VP_PIN_SOURCE  				 GPIO_PinSource9

//PWM VN������Ŷ���   
#define   VN_PIN       					 GPIO_Pin_14
#define   VN_PIN_GROUP  				 GPIOB
#define   VN_PIN_SOURCE  				 GPIO_PinSource14

//PWM WP������Ŷ���   ��TIM1_CH1������
#define   WP_PIN       					 GPIO_Pin_8
#define   WP_PIN_GROUP  				 GPIOA
#define   WP_PIN_SOURCE  				 GPIO_PinSource8

//PWM WN������Ŷ���   ��TIM1_CH1������
#define   WN_PIN       					 GPIO_Pin_13
#define   WN_PIN_GROUP  				 GPIOB
#define   WN_PIN_SOURCE  				 GPIO_PinSource13

//PWM���ͨ������
#define   TIM_Channel_U					 TIM_Channel_3
#define   TIM_Channel_V					 TIM_Channel_2
#define   TIM_Channel_W					 TIM_Channel_1
/************************************************************************************/

/************************UART���Ŷ���**********************************************/
//��ʼ������������ȷ��GPIO���ʱ�Ӵ�
//RXD�������Ŷ���
#define   RXD_PIN       				 GPIO_Pin_7
#define   RXD_PIN_GROUP  				 GPIOB
#define   RXD_PIN_SOURCE  			 GPIO_PinSource7

//TXD�������Ŷ���
#define   TXD_PIN       				 GPIO_Pin_6
#define   TXD_PIN_GROUP  				 GPIOB
#define   TXD_PIN_SOURCE  			 GPIO_PinSource6

#define   RXD2_PIN       				 GPIO_Pin_15
#define   RXD2_PIN_GROUP  				 GPIOA
#define   RXD2_PIN_SOURCE  			 GPIO_PinSource15
//TXD�������Ŷ���
#define   TXD2_PIN       				 GPIO_Pin_14
#define   TXD2_PIN_GROUP  				 GPIOA
#define   TXD2_PIN_SOURCE  			 GPIO_PinSource14
/************************************************************************************/

/************************ADC���Ŷ���**********************************************/
//��ʼ������������ȷ��GPIO���ʱ�Ӵ�
#define   TEMP_CONTROLLER_PIN    						GPIO_Pin_3
#define   TEMP_CONTROLLER_PIN_GROUP  				GPIOA
#define   TEMP_CONTROLLER_PIN_CHANNEL  			ADC_Channel_3


#define   TEMP_MOTOR_PIN    						    GPIO_Pin_1
#define   TEMP_MOTOR_PIN_GROUP  						GPIOA
#define   TEMP_MOTOR_PIN_CHANNEL   					ADC_Channel_1

#define   TORQUE_PIN    						    		GPIO_Pin_2
#define   TORQUE_PIN_GROUP  								GPIOA
#define   TORQUE_PIN_CHANNEL   							ADC_Channel_2

#define   VOLTAGE_PIN    						    		GPIO_Pin_0
#define   VOLTAGE_PIN_GROUP  								GPIOA
#define   VOLTAGE_PIN_CHANNEL   						ADC_Channel_0

#define   CURRENT_W_PIN    						    	GPIO_Pin_4
#define   CURRENT_W_PIN_GROUP  							GPIOA
#define   CURRENT_W_PIN_CHANNEL  						ADC_Channel_4

#define   CURRENT_U_PIN    						    	GPIO_Pin_5
#define   CURRENT_U_PIN_GROUP  							GPIOA
#define   CURRENT_U_PIN_CHANNEL   					ADC_Channel_5

#define   CURRENT_V_PIN    						    	GPIO_Pin_6
#define   CURRENT_V_PIN_GROUP  							GPIOA
#define   CURRENT_V_PIN_CHANNEL  						ADC_Channel_6


//�°涨��

#define   TEMP_CONTROLLER                   3
#define   TORQUE                            4
#define   TEMP_MOTOR                        5
#define   VOLTAGE														6

/*
//�ɰ涨��
#define		CURRENT_V													0
#define		CURRENT_U													1
#define		CURRENT_W													2
#define   VOLTAGE														3
#define   TORQUE                            4
#define   TEMP_MOTOR                        5
#define   TEMP_CONTROLLER                   6
*/

/************************************************************************************/






/*************IO���ŵ�ƽ��ȫ������************************/
extern u8 PIN_Level_power;
extern u8 PIN_Level_light;
extern u8 PIN_Level_brake;

extern u8 PIN_Level_HU;
extern u8 PIN_Level_HV;
extern u8 PIN_Level_HW;

extern u8 PIN_Level_angle_pluse;
extern u8 PIN_Level_over_current;
/*********************************************************/




/*************����������ȫ������************************/
extern volatile s16 I_q;


/***************��ѭ�����Ʊ�־***************************/
extern volatile u8 flag_main_cycle;

/***************ɲ�����Ʊ�־***************************/
extern u8 flag_brake;

/************���ݽ���**********************/
extern volatile u8 flag_new_data_received;
extern volatile u8 flag_mode;
extern uint8_t Flag_test;

/*************�ⲿ��������************************/
extern void GPIO_Config(void);
extern void Timers_Config(void);
extern void ADCWithDMA_Config(void);
extern void UART1_Config(void);
extern void UART2_Config(void);
extern void EXTI_Config(void);
extern void setupTIM1_Interrupt(void);
extern int16_t average(int32_t *sum, int16_t array[], uint8_t length2N, int16_t *index, uint8_t *flag_av_finished, int16_t value);

extern void WWDG_Config(void);
extern void torque_signal_to_Iq_ref(void);

extern void square_wave_drive(void);
extern void PWM_Output(void);




#endif



